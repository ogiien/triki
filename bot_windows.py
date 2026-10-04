#!/usr/bin/env python3
"""Windows Triki BLE peripheral and screen-based obstacle autopilot.

BLE HCI access uses Bumble over a compatible USB Bluetooth adapter. The adapter
must be accessible through WinUSB; see README.md before installing any driver.
"""

from __future__ import annotations

import argparse
import asyncio
import getpass
import logging
import queue
import signal
import struct
import threading
import time
from typing import Any

import cv2
import mss
import numpy as np
from bumble.att import Attribute
from bumble.core import UUID
from bumble.device import Device, DeviceConfiguration
from bumble.gatt import Characteristic, CharacteristicValue, Service
from bumble.hci import Address
from bumble.transport import open_transport


LOG = logging.getLogger("triki_windows")
DEVICE_NAME = "Triki DBD57D"
SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
VENDOR_UUID = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
BATTERY_SERVICE_UUID = "180F"
BATTERY_LEVEL_UUID = "2A19"
DEVICE_INFO_SERVICE_UUID = "180A"
FIRMWARE_REVISION_UUID = "2A26"
FIRMWARE_REVISION = b"3.2.1-A"
FRAME_PERIOD = 0.01
NOTIFICATION_SIZES = (20, 20, 2)
ACCEL_Z = 2048
READY_FRAME = b"\x21\x00\x00\x00\x00"

stop_event = threading.Event()
game_active = threading.Event()
ready_pending = threading.Event()
control_queue: queue.Queue[int] = queue.Queue(maxsize=1)
frame_queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)


def publish_latest(target: queue.Queue, value: Any) -> None:
    try:
        target.put_nowait(value)
    except queue.Full:
        try:
            target.get_nowait()
        except queue.Empty:
            pass
        target.put_nowait(value)


def discard_pending(target: queue.Queue) -> None:
    while True:
        try:
            target.get_nowait()
        except queue.Empty:
            return


def parse_static_address(address: str) -> Address:
    parts = address.strip().replace("-", ":").split(":")
    if len(parts) != 6:
        raise ValueError("Adres musi mieć format XX:XX:XX:XX:XX:XX")
    try:
        octets = [int(part, 16) for part in parts]
    except ValueError as error:
        raise ValueError("Adres zawiera nieprawidłowy znak szesnastkowy") from error
    if any(not 0 <= octet <= 0xFF for octet in octets):
        raise ValueError("Każdy oktet adresu musi być z zakresu 00–FF")
    if octets[0] & 0xC0 != 0xC0:
        raise ValueError("Adres kapsla nie wygląda na BLE random-static (MSB musi zaczynać się od C–F)")
    return Address(":".join(f"{octet:02X}" for octet in octets))


def make_imu_frame(accel_x: int) -> bytes:
    return struct.pack(
        "<BB6h",
        0x22,
        0x00,
        0,
        0,
        0,
        max(-32768, min(32767, int(accel_x))),
        0,
        ACCEL_Z,
    )


def split_notifications(stream: bytes | bytearray) -> list[bytes]:
    if len(stream) != sum(NOTIFICATION_SIZES):
        raise ValueError("Strumień musi zawierać 3 ramki IMU, czyli 42 bajty")
    chunks = []
    offset = 0
    for size in NOTIFICATION_SIZES:
        chunks.append(bytes(stream[offset : offset + size]))
        offset += size
    return chunks


def make_advertising_data() -> tuple[bytes, bytes]:
    name = DEVICE_NAME.encode("ascii")
    advertising = bytes((2, 0x01, 0x06, 5, 0xFF, 0x00, 0xFF, 0xA0, 0x0A))
    advertising += bytes((len(name) + 1, 0x09)) + name
    scan_response = bytes((3, 0x03, 0x01, 0x00))
    return advertising, scan_response


def build_gatt(device: Device) -> tuple[Characteristic, Characteristic]:
    def on_rx_write(_connection: Any, value: bytes) -> None:
        payload = bytes(value)
        if payload[:2] == b"\x20\x10":
            discard_pending(control_queue)
            ready_pending.set()
            game_active.set()
            LOG.info("RX START: autopilot aktywny")
        elif payload[:2] == b"\x20\x00":
            game_active.clear()
            ready_pending.clear()
            discard_pending(control_queue)
            publish_latest(control_queue, 0)
            LOG.info("RX STOP: autopilot wstrzymany")

    led_value = bytearray(b"\x00")

    def read_led(_connection: Any) -> bytes:
        return bytes(led_value)

    def write_led(_connection: Any, value: bytes) -> None:
        led_value[:] = bytes((bytes(value)[0] & 0x01,)) if value else b"\x00"

    nus = Service(
        SERVICE_UUID,
        [
            Characteristic(
                RX_UUID,
                Characteristic.Properties.WRITE
                | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
                Attribute.Permissions.WRITEABLE,
                CharacteristicValue(write=on_rx_write),
            ),
            Characteristic(
                TX_UUID,
                Characteristic.Properties.NOTIFY,
                Attribute.Permissions(0),
                None,
            ),
            Characteristic(
                VENDOR_UUID,
                Characteristic.Properties.READ | Characteristic.Properties.WRITE,
                Attribute.Permissions.READABLE | Attribute.Permissions.WRITEABLE,
                CharacteristicValue(read=read_led, write=write_led),
            ),
        ],
    )
    battery = Service(
        BATTERY_SERVICE_UUID,
        [
            Characteristic(
                BATTERY_LEVEL_UUID,
                Characteristic.Properties.READ | Characteristic.Properties.NOTIFY,
                Attribute.Permissions.READABLE,
                CharacteristicValue(read=lambda _connection: b"\x64"),
            )
        ],
    )
    device_information = Service(
        DEVICE_INFO_SERVICE_UUID,
        [
            Characteristic(
                FIRMWARE_REVISION_UUID,
                Characteristic.Properties.READ,
                Attribute.Permissions.READABLE,
                FIRMWARE_REVISION,
            )
        ],
    )
    device.add_services([nus, battery, device_information])
    tx_characteristic = next(
        characteristic
        for characteristic in nus.characteristics
        if characteristic.uuid == UUID(TX_UUID)
    )
    battery_characteristic = next(
        characteristic
        for characteristic in battery.characteristics
        if characteristic.uuid == UUID(BATTERY_LEVEL_UUID)
    )
    return tx_characteristic, battery_characteristic


def _detect_clouds(mask: np.ndarray, minimum_area: float) -> list[tuple[int, int, int, int]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [cv2.boundingRect(contour) for contour in contours if cv2.contourArea(contour) >= minimum_area]


def _estimate_fall_speed(
    cloud: tuple[int, int, int, int],
    previous: list[tuple[int, int, int, int]],
    elapsed: float,
    frame_width: int,
    frame_height: int,
) -> float:
    if elapsed <= 0:
        return 0.0
    x, y, width, height = cloud
    center_x = x + width / 2
    center_y = y + height / 2
    matches = []
    for old_x, old_y, old_width, old_height in previous:
        dx = abs(center_x - (old_x + old_width / 2))
        dy = center_y - (old_y + old_height / 2)
        if dx <= max(40, frame_width * 0.12) and -frame_height * 0.02 <= dy <= frame_height * 0.15:
            matches.append((dx + abs(dy) * 0.25, dy))
    if not matches:
        return 0.0
    return max(0.0, min(frame_height * 2, min(matches)[1] / elapsed))


def _capture_region(screen: mss.mss, args: argparse.Namespace) -> dict[str, int]:
    if args.region:
        left, top, width, height = args.region
        if width <= 0 or height <= 0:
            raise ValueError("Szerokość i wysokość --region muszą być dodatnie")
        return {"left": left, "top": top, "width": width, "height": height}
    if not 0 < args.monitor < len(screen.monitors):
        raise ValueError(f"Brak monitora MSS o indeksie {args.monitor}")
    return dict(screen.monitors[args.monitor])


def vision_worker(args: argparse.Namespace, prediction_seconds: float) -> None:
    try:
        previous_clouds: list[tuple[int, int, int, int]] = []
        previous_time: float | None = None
        kernel = np.ones((3, 3), dtype=np.uint8)
        with mss.mss() as screen:
            region = _capture_region(screen, args)
            LOG.info("MSS region: %s; wizja czeka na RX START", region)
            while not stop_event.is_set():
                if not game_active.wait(0.25):
                    previous_clouds = []
                    previous_time = None
                    continue
                while game_active.is_set() and not stop_event.is_set():
                    raw = np.asarray(screen.grab(region))
                    if raw.size == 0:
                        raise RuntimeError("MSS zwrócił pustą klatkę")
                    frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
                    source_height, source_width = frame.shape[:2]
                    scale = min(1.0, 960 / source_width, 720 / source_height)
                    if scale < 1.0:
                        frame = cv2.resize(
                            frame,
                            (int(source_width * scale), int(source_height * scale)),
                            interpolation=cv2.INTER_AREA,
                        )
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    height, width = frame.shape[:2]
                    now = time.monotonic()
                    elapsed = now - previous_time if previous_time is not None else 0.0
                    ship_x = width // 2
                    ship_y = int(height * 0.88)
                    ship_half_width = max(12, int(width * 0.035))
                    mask = cv2.inRange(
                        hsv,
                        np.array((125, 50, 50), dtype=np.uint8),
                        np.array((160, 255, 255), dtype=np.uint8),
                    )
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                    clouds = _detect_clouds(mask, max(50, width * height * 0.00006))

                    projected = []
                    max_speed = 0.0
                    for cloud in clouds:
                        speed = _estimate_fall_speed(cloud, previous_clouds, elapsed, width, height)
                        max_speed = max(max_speed, speed)
                        x, y, cloud_width, cloud_height = cloud
                        projected.append((x, y + int(speed * prediction_seconds), cloud_width, cloud_height))
                    lookahead = max(
                        int(height * 0.52),
                        int(max_speed * prediction_seconds) + ship_half_width * 2,
                    )
                    hazards = [
                        box for box in projected
                        if box[1] < ship_y and box[1] + box[3] >= ship_y - lookahead
                    ]
                    center_threats = [
                        box for box in hazards
                        if box[0] < ship_x + ship_half_width
                        and box[0] + box[2] > ship_x - ship_half_width
                    ]
                    nearest = max(center_threats, key=lambda box: box[1] + box[3], default=None)
                    target = 0
                    direction = "TOR WOLNY"
                    candidates = {}
                    if nearest:
                        step = max(int(width * 0.18), ship_half_width * 3)
                        candidates = {
                            "SKRĘT W LEWO": max(ship_half_width, ship_x - step),
                            "SKRĘT W PRAWO": min(width - ship_half_width, ship_x + step),
                        }

                        def clearance(candidate_x: int) -> float:
                            return min(
                                (
                                    abs(candidate_x - (x + cloud_width / 2))
                                    - cloud_width / 2
                                    - ship_half_width
                                    for x, _y, cloud_width, _h in hazards
                                ),
                                default=float(width),
                            )

                        direction = max(candidates, key=lambda side: clearance(candidates[side]))
                        target = -12000 if direction == "SKRĘT W LEWO" else 12000
                        cv2.arrowedLine(
                            frame,
                            (ship_x, ship_y),
                            (candidates[direction], ship_y - 60),
                            (0, 255, 255),
                            2,
                        )

                    for (x, y, cloud_width, cloud_height), predicted in zip(clouds, projected):
                        is_nearest = nearest is not None and predicted == nearest
                        color = (255, 0, 255) if is_nearest else (180, 80, 210)
                        cv2.rectangle(frame, (x, y), (x + cloud_width, y + cloud_height), color, 2 if is_nearest else 1)
                    cv2.rectangle(
                        frame,
                        (ship_x - ship_half_width, ship_y - 12),
                        (ship_x + ship_half_width, ship_y + 12),
                        (40, 220, 80),
                        2,
                    )
                    cv2.putText(
                        frame,
                        f"{direction} | accel X={target} | lookahead={lookahead}px",
                        (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.58,
                        (255, 255, 255),
                        2,
                    )
                    publish_latest(control_queue, target)
                    publish_latest(frame_queue, frame)
                    previous_clouds = clouds
                    previous_time = now
                    if stop_event.wait(1 / 30):
                        break
    except Exception:
        LOG.exception("Wątek wizji zakończył działanie")
        stop_event.set()


async def notify_loop(
    device: Device,
    tx_characteristic: Characteristic,
    battery_characteristic: Characteristic,
) -> None:
    loop = asyncio.get_running_loop()
    next_tick = loop.time()
    stream = bytearray()
    current_accel = 0
    last_battery = loop.time()
    while not stop_event.is_set():
        try:
            current_accel = control_queue.get_nowait()
        except queue.Empty:
            pass
        if not game_active.is_set():
            stream.clear()
            next_tick = loop.time()
            await asyncio.sleep(0.02)
            continue
        if ready_pending.is_set():
            await device.gatt_server.notify_subscribers(tx_characteristic, READY_FRAME)
            ready_pending.clear()
            next_tick = loop.time()
            continue

        stream.extend(make_imu_frame(current_accel))
        if len(stream) == 42:
            for chunk in split_notifications(stream):
                await device.gatt_server.notify_subscribers(tx_characteristic, chunk)
            stream.clear()
        if loop.time() - last_battery >= 5.0:
            await device.gatt_server.notify_subscribers(battery_characteristic, b"\x64")
            last_battery = loop.time()
        next_tick += FRAME_PERIOD
        await asyncio.sleep(max(0.0, next_tick - loop.time()))


async def run(args: argparse.Namespace, address: Address) -> None:
    advertising, scan_response = make_advertising_data()
    config = DeviceConfiguration(
        name=DEVICE_NAME,
        address=address,
        le_privacy_enabled=False,
        classic_enabled=False,
        advertising_data=advertising,
        scan_response_data=scan_response,
        advertising_interval_min=30.0,
        advertising_interval_max=45.0,
    )
    async with await open_transport(args.transport) as transport:
        device = Device.from_config_with_hci(config, transport.source, transport.sink)
        tx, battery = build_gatt(device)
        await device.power_on()
        LOG.info("Radio HCI aktywne; MAC random-static: %s", address)
        await device.start_advertising(auto_restart=True)
        LOG.info("Reklamuję %s; scan response UUID 0x0001", DEVICE_NAME)

        vision = threading.Thread(
            target=vision_worker,
            args=(args, (args.latency_ms + args.reaction_ms) / 1000.0),
            name="triki-vision",
            daemon=True,
        )
        vision.start()
        sender = asyncio.create_task(notify_loop(device, tx, battery))
        cv2.namedWindow("Triki Windows autopilot", cv2.WINDOW_NORMAL)
        waiting = np.zeros((240, 460, 3), dtype=np.uint8)
        cv2.putText(
            waiting,
            "Czekam na RX START z telefonu",
            (15, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (230, 230, 230),
            2,
        )
        cv2.imshow("Triki Windows autopilot", waiting)
        try:
            while not stop_event.is_set():
                try:
                    frame = frame_queue.get_nowait()
                except queue.Empty:
                    frame = None
                if frame is not None:
                    height, width = frame.shape[:2]
                    scale = min(1.0, 960 / width, 720 / height)
                    if scale < 1.0:
                        frame = cv2.resize(
                            frame,
                            (int(width * scale), int(height * scale)),
                            interpolation=cv2.INTER_AREA,
                        )
                    cv2.imshow("Triki Windows autopilot", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    stop_event.set()
                await asyncio.sleep(0.002)
        finally:
            stop_event.set()
            game_active.clear()
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
            await device.stop_advertising()
            vision.join(timeout=2.0)
            cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", default="usb:0", help="Bumble HCI transport, np. usb:0")
    parser.add_argument("--address", help="Random-static MAC żetonu; bez argumentu będzie ukryty prompt")
    parser.add_argument("--monitor", type=int, default=1, help="Indeks monitora MSS (domyślnie ekran główny)")
    parser.add_argument("--region", nargs=4, type=int, metavar=("LEFT", "TOP", "WIDTH", "HEIGHT"))
    parser.add_argument("--latency-ms", type=float, default=120)
    parser.add_argument("--reaction-ms", type=float, default=180)
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    if args.latency_ms < 0 or args.reaction_ms < 0:
        raise SystemExit("Opóźnienia nie mogą być ujemne")
    address_text = args.address
    if not address_text:
        address_text = getpass.getpass("MAC random-static kapsla (wpis nie będzie widoczny): ")
    try:
        address = parse_static_address(address_text)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    signal.signal(signal.SIGINT, lambda _signum, _frame: stop_event.set())
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, lambda _signum, _frame: stop_event.set())
    try:
        asyncio.run(run(args, address))
    except KeyboardInterrupt:
        stop_event.set()
    except Exception:
        LOG.exception("Bot nie wystartował. Sprawdź adapter BLE HCI i sterownik WinUSB.")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
