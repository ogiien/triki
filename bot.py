#!/usr/bin/env python3
"""BLE peripheral and OpenCV autopilot for a QuickTime-mirrored game on macOS."""

# Protocol behavior adapted from Maku-hub/TrikiEmu, MIT license; see TrikiEmu/LICENSE.

from __future__ import annotations

import argparse
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
import objc
import Quartz
from CoreBluetooth import (
    CBATTErrorSuccess,
    CBAttributePermissionsReadable,
    CBAdvertisementDataLocalNameKey,
    CBAdvertisementDataServiceUUIDsKey,
    CBAttributePermissionsWriteable,
    CBCharacteristicPropertyNotify,
    CBCharacteristicPropertyRead,
    CBCharacteristicPropertyWrite,
    CBCharacteristicPropertyWriteWithoutResponse,
    CBManagerStatePoweredOn,
    CBMutableCharacteristic,
    CBMutableService,
    CBPeripheralManager,
    CBUUID,
)
from CoreFoundation import CFRunLoopRunInMode, kCFRunLoopDefaultMode
from Foundation import NSData, NSObject, NSTimer


LOG = logging.getLogger("triki_bot")
DEVICE_NAME = "Triki DBD57D"
SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
VENDOR_UUID = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
BATTERY_SERVICE_UUID = "180F"
BATTERY_LEVEL_UUID = "2A19"
DEVICE_INFO_SERVICE_UUID = "180A"
FIRMWARE_REVISION_UUID = "2A26"
FIRMWARE_REVISION = "3.2.1-A"
FRAME_PERIOD = 0.01
STREAM_NOTIFY_SIZES = (20, 20, 2)
ACCEL_Z = 2048

stop_event = threading.Event()
game_active = threading.Event()
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


def make_imu_frame(lateral_acceleration: int) -> bytes:
    """Pack the TrikiEmu frame layout: header, status, gyro XYZ and accel XYZ."""
    return struct.pack(
        "<BB6h",
        0x22,
        0x00,
        0,
        0,
        0,
        max(-32768, min(32767, lateral_acceleration)),
        0,
        ACCEL_Z,
    )


def split_stream_notifications(stream: bytes | bytearray) -> list[bytes]:
    if len(stream) != sum(STREAM_NOTIFY_SIZES):
        raise ValueError("Strumień musi zawierać dokładnie trzy ramki IMU (42 B)")
    chunks = []
    offset = 0
    for size in STREAM_NOTIFY_SIZES:
        chunks.append(bytes(stream[offset : offset + size]))
        offset += size
    return chunks


def normalize_bgr_frame(raw: np.ndarray) -> np.ndarray:
    if raw.ndim != 3:
        raise ValueError(f"Nieprawidłowy wymiar klatki: {raw.shape}")
    if raw.shape[2] == 3:
        return np.ascontiguousarray(raw)
    if raw.shape[2] == 4:
        return cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
    raise ValueError(f"Nieobsługiwana liczba kanałów klatki: {raw.shape[2]}")


class PeripheralDelegate(NSObject):
    def init(self) -> "PeripheralDelegate":
        self = objc.super(PeripheralDelegate, self).init()
        if self is None:
            return None
        self.manager = None
        self.advertisement_data = None
        self.rx_characteristic = None
        self.tx_characteristic = None
        self.vendor_characteristic = None
        self.vendor_value = b"\x00"
        self.battery_characteristic = None
        self.battery_value = b"\x64"
        self.firmware_characteristic = None
        self.ready_timer = None
        self.tx_subscribed = False
        self.battery_subscribed = False
        self.is_streaming = False
        self.ready_pending = False
        self.stream_buffer = bytearray()
        self.pending_chunks: list[bytes] = []
        self.services_pending = 0
        self.nus_service_added = False
        self.rejected_services: list[str] = []
        self.last_battery_update = time.monotonic()
        self.battery_update_pending = False
        self.target_accel_x = 0
        self.current_accel_x = 0
        self.did_configure = False
        return self

    def peripheralManagerDidUpdateState_(self, manager: Any) -> None:
        state = manager.state()
        LOG.info("CoreBluetooth state: %s", state)
        if state != CBManagerStatePoweredOn or self.did_configure:
            return

        self.did_configure = True
        self.manager = manager
        service = CBMutableService.alloc().initWithType_primary_(
            CBUUID.UUIDWithString_(SERVICE_UUID), True
        )
        self.rx_characteristic = (
            CBMutableCharacteristic.alloc().initWithType_properties_value_permissions_(
                CBUUID.UUIDWithString_(RX_UUID),
                CBCharacteristicPropertyWrite | CBCharacteristicPropertyWriteWithoutResponse,
                None,
                CBAttributePermissionsWriteable,
            )
        )
        self.tx_characteristic = (
            CBMutableCharacteristic.alloc().initWithType_properties_value_permissions_(
                CBUUID.UUIDWithString_(TX_UUID),
                CBCharacteristicPropertyNotify,
                None,
                0,
            )
        )
        self.vendor_characteristic = (
            CBMutableCharacteristic.alloc().initWithType_properties_value_permissions_(
                CBUUID.UUIDWithString_(VENDOR_UUID),
                CBCharacteristicPropertyRead | CBCharacteristicPropertyWrite,
                None,
                CBAttributePermissionsReadable | CBAttributePermissionsWriteable,
            )
        )
        service.setCharacteristics_(
            [self.rx_characteristic, self.tx_characteristic, self.vendor_characteristic]
        )

        battery_service = CBMutableService.alloc().initWithType_primary_(
            CBUUID.UUIDWithString_(BATTERY_SERVICE_UUID), True
        )
        self.battery_characteristic = (
            CBMutableCharacteristic.alloc().initWithType_properties_value_permissions_(
                CBUUID.UUIDWithString_(BATTERY_LEVEL_UUID),
                CBCharacteristicPropertyRead | CBCharacteristicPropertyNotify,
                None,
                CBAttributePermissionsReadable,
            )
        )
        battery_service.setCharacteristics_([self.battery_characteristic])

        device_info_service = CBMutableService.alloc().initWithType_primary_(
            CBUUID.UUIDWithString_(DEVICE_INFO_SERVICE_UUID), True
        )
        firmware_value = FIRMWARE_REVISION.encode("ascii")
        self.firmware_characteristic = (
            CBMutableCharacteristic.alloc().initWithType_properties_value_permissions_(
                CBUUID.UUIDWithString_(FIRMWARE_REVISION_UUID),
                CBCharacteristicPropertyRead,
                NSData.dataWithBytes_length_(firmware_value, len(firmware_value)),
                CBAttributePermissionsReadable,
            )
        )
        device_info_service.setCharacteristics_([self.firmware_characteristic])

        self.services_pending = 3
        for gatt_service in (service, battery_service, device_info_service):
            manager.addService_(gatt_service)

    def peripheralManager_didAddService_error_(
        self, manager: Any, service: Any, error: Any
    ) -> None:
        service_uuid = str(service.UUID().UUIDString()).upper()
        if error is not None:
            LOG.warning("CoreBluetooth odrzucił usługę %s: %s", service_uuid, error)
            self.rejected_services.append(service_uuid)
            if service_uuid == SERVICE_UUID:
                LOG.error("Wymagana usługa NUS nie została dodana")
                stop_event.set()
                return
        elif service_uuid == SERVICE_UUID:
            self.nus_service_added = True

        self.services_pending -= 1
        if self.services_pending == 0:
            if not self.nus_service_added:
                LOG.error("Brak wymaganej usługi NUS; nie uruchamiam reklamy")
                stop_event.set()
                return
            if self.rejected_services:
                LOG.warning(
                    "Brakuje usług opcjonalnych; Żappka może ich wymagać: %s",
                    ", ".join(self.rejected_services),
                )
            # CoreBluetooth has no API for manufacturer data or a separate scan response.
            LOG.warning(
                "CoreBluetooth nie ustawia MAC i nie emuluje manufacturer data ani osobnego scan response"
            )
            self.advertisement_data = {
                CBAdvertisementDataLocalNameKey: DEVICE_NAME,
                CBAdvertisementDataServiceUUIDsKey: [
                    CBUUID.UUIDWithString_("0001"),
                ],
            }
            manager.startAdvertising_(self.advertisement_data)

    def peripheralManagerDidStartAdvertising_error_(
        self, manager: Any, error: Any
    ) -> None:
        if error is not None:
            LOG.error("Nie udało się rozpocząć reklamy BLE: %s", error)
            stop_event.set()
            return
        LOG.info("BLE reklamuje %s", DEVICE_NAME)

    def peripheralManager_central_didSubscribeToCharacteristic_(
        self, manager: Any, central: Any, characteristic: Any
    ) -> None:
        uuid = str(characteristic.UUID().UUIDString()).upper()
        if uuid == TX_UUID:
            self.tx_subscribed = True
            LOG.info("Centrala zasubskrybowała TX")
        elif uuid == BATTERY_LEVEL_UUID:
            self.battery_subscribed = True

    def peripheralManager_central_didUnsubscribeFromCharacteristic_(
        self, manager: Any, central: Any, characteristic: Any
    ) -> None:
        uuid = str(characteristic.UUID().UUIDString()).upper()
        if uuid == TX_UUID:
            self.tx_subscribed = False
            self.is_streaming = False
            game_active.clear()
            discard_pending(control_queue)
            self.target_accel_x = 0
            self.stream_buffer.clear()
            self.pending_chunks.clear()
            LOG.info("Centrala anulowała subskrypcję TX")
            if self.advertisement_data is not None:
                manager.startAdvertising_(self.advertisement_data)
        elif uuid == BATTERY_LEVEL_UUID:
            self.battery_subscribed = False

    def peripheralManager_didReceiveWriteRequests_(
        self, manager: Any, requests: Any
    ) -> None:
        for request in requests:
            uuid = str(request.characteristic().UUID().UUIDString()).upper()
            value = bytes(request.value() or b"")
            if uuid == RX_UUID:
                if value[:2] == b"\x20\x10":
                    self.is_streaming = True
                    self.ready_pending = True
                    self.stream_buffer.clear()
                    self.pending_chunks.clear()
                    discard_pending(control_queue)
                    self.target_accel_x = 0
                    game_active.set()
                    LOG.info("RX: START")
                elif value[:2] == b"\x20\x00":
                    self.is_streaming = False
                    game_active.clear()
                    discard_pending(control_queue)
                    self.target_accel_x = 0
                    self.stream_buffer.clear()
                    self.pending_chunks.clear()
                    LOG.info("RX: STOP")
            elif uuid == VENDOR_UUID and value:
                self.vendor_value = bytes((value[0] & 0x01,))
        if requests:
            manager.respondToRequest_withResult_(requests[0], CBATTErrorSuccess)

    def peripheralManager_didReceiveReadRequest_(
        self, manager: Any, request: Any
    ) -> None:
        uuid = str(request.characteristic().UUID().UUIDString()).upper()
        if uuid == VENDOR_UUID:
            data = self.vendor_value
        elif uuid == BATTERY_LEVEL_UUID:
            data = self.battery_value
        else:
            return
        request.setValue_(NSData.dataWithBytes_length_(data, len(data)))
        manager.respondToRequest_withResult_(request, CBATTErrorSuccess)

    def peripheralManagerIsReadyToUpdateSubscribers_(self, manager: Any) -> None:
        pass

    def start_timer(self) -> None:
        selector = objc.selector(self.timerFired_, signature=b"v@:@")
        self.ready_timer = NSTimer.scheduledTimerWithTimeInterval_target_selector_userInfo_repeats_(
            FRAME_PERIOD, self, selector, None, True
        )

    def timerFired_(self, timer: Any) -> None:
        if self.manager is None:
            return

        try:
            self.target_accel_x = control_queue.get_nowait()
        except queue.Empty:
            pass
        if not self.is_streaming:
            self.target_accel_x = 0
        self.current_accel_x = int(
            self.current_accel_x + (self.target_accel_x - self.current_accel_x) * 0.18
        )

        now = time.monotonic()
        if self.battery_subscribed and now - self.last_battery_update >= 5.0:
            self.last_battery_update = now
            self.battery_update_pending = True
        if self.battery_update_pending and self.battery_subscribed:
            battery = NSData.dataWithBytes_length_(
                self.battery_value, len(self.battery_value)
            )
            self.battery_characteristic.setValue_(battery)
            if self.manager.updateValue_forCharacteristic_onSubscribedCentrals_(
                battery, self.battery_characteristic, None
            ):
                self.battery_update_pending = False

        if not self.tx_subscribed or not self.is_streaming:
            return
        if self.ready_pending:
            payload = b"\x21\x00\x00\x00\x00"
            data = NSData.dataWithBytes_length_(payload, len(payload))
            if self.manager.updateValue_forCharacteristic_onSubscribedCentrals_(
                data, self.tx_characteristic, None
            ):
                self.ready_pending = False
            return

        if not self.pending_chunks and len(self.stream_buffer) < 42:
            self.stream_buffer.extend(make_imu_frame(self.current_accel_x))
            if len(self.stream_buffer) == 42:
                self.pending_chunks = split_stream_notifications(self.stream_buffer)
                self.stream_buffer.clear()

        while self.pending_chunks:
            chunk = self.pending_chunks[0]
            data = NSData.dataWithBytes_length_(chunk, len(chunk))
            if not self.manager.updateValue_forCharacteristic_onSubscribedCentrals_(
                data, self.tx_characteristic, None
            ):
                break
            self.pending_chunks.pop(0)


def _detect_clouds(mask: np.ndarray, minimum_area: float) -> list[tuple[int, int, int, int]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for contour in contours:
        if cv2.contourArea(contour) < minimum_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        detections.append((x, y, width, height))
    return detections


def _vertical_speed(
    cloud: tuple[int, int, int, int],
    previous: list[tuple[int, int, int, int]],
    elapsed: float,
    frame_width: int,
    frame_height: int,
) -> float:
    if elapsed <= 0 or not previous:
        return 0.0

    x, y, width, height = cloud
    center_x = x + width / 2
    center_y = y + height / 2
    candidates = []
    for old_x, old_y, old_width, old_height in previous:
        old_center_x = old_x + old_width / 2
        old_center_y = old_y + old_height / 2
        dx = abs(center_x - old_center_x)
        dy = center_y - old_center_y
        if dx <= max(40, frame_width * 0.12) and -frame_height * 0.02 <= dy <= frame_height * 0.15:
            candidates.append((dx + abs(dy) * 0.25, dy))
    if not candidates:
        return 0.0

    _, vertical_delta = min(candidates)
    return max(0.0, min(frame_height * 2.0, vertical_delta / elapsed))


class QuickTimeWindowCapture:
    def __init__(self) -> None:
        self.window_id: int | None = None
        self.last_search = 0.0

    def _find_window(self) -> int | None:
        windows = Quartz.CGWindowListCopyWindowInfo(
            Quartz.kCGWindowListOptionOnScreenOnly,
            Quartz.kCGNullWindowID,
        )
        matches = []
        for window in windows or []:
            owner = str(window.get("kCGWindowOwnerName", "")).lower()
            bounds = window.get("kCGWindowBounds", {})
            if "quicktime player" not in owner or window.get("kCGWindowLayer", -1) != 0:
                continue
            width = int(bounds.get("Width", 0))
            height = int(bounds.get("Height", 0))
            if width < 320 or height < 240:
                continue
            matches.append((width * height, int(window["kCGWindowNumber"])))
        return max(matches)[1] if matches else None

    def grab(self) -> np.ndarray:
        now = time.monotonic()
        if self.window_id is None or now - self.last_search > 2.0:
            self.window_id = self._find_window()
            self.last_search = now
        if self.window_id is None:
            raise RuntimeError(
                "Nie znaleziono widocznego okna QuickTime Player z obrazem iPhone'a"
            )

        image = Quartz.CGWindowListCreateImage(
            Quartz.CGRectNull,
            Quartz.kCGWindowListOptionIncludingWindow,
            self.window_id,
            Quartz.kCGWindowImageBoundsIgnoreFraming,
        )
        if image is None:
            self.window_id = None
            raise RuntimeError(
                "Nie można przechwycić okna QuickTime; sprawdź uprawnienie Nagrywanie ekranu"
            )

        width = Quartz.CGImageGetWidth(image)
        height = Quartz.CGImageGetHeight(image)
        bytes_per_row = Quartz.CGImageGetBytesPerRow(image)
        bits_per_pixel = Quartz.CGImageGetBitsPerPixel(image)
        if bits_per_pixel != 32 or bytes_per_row < width * 4:
            raise RuntimeError(
                f"Nieobsługiwany format obrazu QuickTime: {bits_per_pixel} bpp"
            )

        provider_data = Quartz.CGDataProviderCopyData(
            Quartz.CGImageGetDataProvider(image)
        )
        pixels = np.frombuffer(bytes(provider_data), dtype=np.uint8)
        pixels = pixels.reshape(height, bytes_per_row)[:, : width * 4]
        pixels = pixels.reshape(height, width, 4)

        bitmap_info = int(Quartz.CGImageGetBitmapInfo(image))
        byte_order = bitmap_info & 0x7000
        alpha_info = bitmap_info & 0x1F
        if byte_order == 0x2000:
            bgr = pixels[:, :, :3]
        elif alpha_info in (2, 4, 6):
            bgr = pixels[:, :, 1:4][:, :, ::-1]
        else:
            bgr = pixels[:, :, :3][:, :, ::-1]
        return np.ascontiguousarray(bgr)


def vision_worker(
    region: tuple[int, int, int, int] | None,
    prediction_seconds: float,
    control_output: queue.Queue[int],
    frame_output: queue.Queue[np.ndarray],
) -> None:
    try:
        previous_clouds: list[tuple[int, int, int, int]] = []
        previous_time: float | None = None
        morphology_kernel = np.ones((3, 3), dtype=np.uint8)
        LOG.info("Wątek wizji gotowy; czeka na RX START")

        screen_context = mss.mss() if region is not None else None
        capture = (
            screen_context
            if region is not None
            else QuickTimeWindowCapture()
        )
        if screen_context is not None:
            left, top, region_width, region_height = region
            monitor = {
                "left": left,
                "top": top,
                "width": region_width,
                "height": region_height,
            }

        try:
            while not stop_event.is_set():
                if not game_active.wait(timeout=0.25):
                    previous_clouds = []
                    previous_time = None
                    continue

                while game_active.is_set() and not stop_event.is_set():
                    raw = np.asarray(
                        capture.grab(monitor) if region is not None else capture.grab()
                    )
                    if raw.size == 0:
                        raise RuntimeError("MSS zwrócił pustą klatkę")
                    frame = normalize_bgr_frame(raw)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    height, width = frame.shape[:2]
                    now = time.monotonic()
                    elapsed = now - previous_time if previous_time is not None else 0.0
                    center_x = width // 2
                    ship_y = int(height * 0.88)
                    ship_half_width = max(12, int(width * 0.035))

                    purple_mask = cv2.inRange(
                        hsv,
                        np.array([125, 50, 50], dtype=np.uint8),
                        np.array([160, 255, 255], dtype=np.uint8),
                    )
                    purple_mask = cv2.morphologyEx(
                        purple_mask,
                        cv2.MORPH_OPEN,
                        morphology_kernel,
                    )
                    clouds = _detect_clouds(
                        purple_mask, max(50.0, width * height * 0.00006)
                    )

                    prediction_delay = prediction_seconds
                    projected = []
                    max_fall_speed = 0.0
                    for cloud in clouds:
                        speed = _vertical_speed(
                            cloud, previous_clouds, elapsed, width, height
                        )
                        max_fall_speed = max(max_fall_speed, speed)
                        x, y, cloud_width, cloud_height = cloud
                        projected_y = y + int(speed * prediction_delay)
                        projected.append((x, projected_y, cloud_width, cloud_height, speed))

                    lookahead = max(
                        int(height * 0.52),
                        int(max_fall_speed * prediction_delay) + ship_half_width * 2,
                    )
                    margin = max(12, int(width * 0.02))
                    future_hazards = [
                        item
                        for item in projected
                        if item[1] < ship_y
                        and item[1] + item[3] >= ship_y - lookahead
                    ]
                    center_threats = [
                        item
                        for item in future_hazards
                        if item[0] < center_x + ship_half_width
                        and item[0] + item[2] > center_x - ship_half_width
                    ]
                    nearest = max(
                        center_threats,
                        key=lambda item: item[1] + item[3],
                        default=None,
                    )

                    target_accel = 0
                    direction = "TOR WOLNY"
                    if nearest is not None:
                        step = max(int(width * 0.18), ship_half_width * 3 + margin)
                        candidates = {
                            "SKRĘT W LEWO": max(ship_half_width + margin, center_x - step),
                            "SKRĘT W PRAWO": min(width - ship_half_width - margin, center_x + step),
                        }

                        def clearance(candidate_x: int) -> float:
                            gaps = [
                                abs(candidate_x - (x + cloud_width / 2))
                                - cloud_width / 2
                                - ship_half_width
                                for x, y, cloud_width, cloud_height, _ in future_hazards
                            ]
                            return min(gaps, default=float(width))

                        direction = max(candidates, key=lambda label: clearance(candidates[label]))
                        target_accel = -12000 if direction == "SKRĘT W LEWO" else 12000
                        cv2.arrowedLine(
                            frame,
                            (center_x, ship_y),
                            (candidates[direction], ship_y - 65),
                            (0, 255, 255),
                            2,
                            tipLength=0.18,
                        )

                    for index, (x, y, cloud_width, cloud_height) in enumerate(clouds):
                        _, projected_y, _, _, _ = projected[index]
                        is_nearest = (
                            nearest is not None
                            and x == nearest[0]
                            and cloud_width == nearest[2]
                            and cloud_height == nearest[3]
                        )
                        cv2.rectangle(
                            frame,
                            (x, y),
                            (x + cloud_width, y + cloud_height),
                            (255, 0, 255) if is_nearest else (180, 80, 210),
                            2 if is_nearest else 1,
                        )
                        if is_nearest:
                            cv2.circle(
                                frame,
                                (x + cloud_width // 2, projected_y + cloud_height // 2),
                                5,
                                (0, 255, 255),
                                -1,
                            )

                    ship_box = (
                        center_x - ship_half_width,
                        ship_y - 12,
                        center_x + ship_half_width,
                        ship_y + 12,
                    )
                    cv2.rectangle(frame, ship_box[:2], ship_box[2:], (40, 220, 80), 2)
                    cv2.putText(
                        frame,
                        f"{direction} | cel X={target_accel} | lookahead={lookahead}px",
                        (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    publish_latest(control_output, target_accel)
                    publish_latest(frame_output, frame)
                    previous_clouds = clouds
                    previous_time = now
                    if stop_event.wait(1 / 30):
                        break
        finally:
            if screen_context is not None:
                screen_context.close()
    except Exception:
        LOG.exception("Wątek wizji zakończył się błędem")
        stop_event.set()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--region",
        nargs=4,
        type=int,
        metavar=("LEFT", "TOP", "WIDTH", "HEIGHT"),
        help="Opcjonalny wycinek MSS; domyślnie bot automatycznie przechwytuje okno QuickTime Player",
    )
    parser.add_argument(
        "--latency-ms",
        type=float,
        default=120.0,
        help="Szacowane opóźnienie klonowania obrazu w milisekundach",
    )
    parser.add_argument(
        "--reaction-ms",
        type=float,
        default=180.0,
        help="Dodatkowy horyzont na reakcję statku w milisekundach",
    )
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    region = tuple(args.region) if args.region else None
    if region is not None and (region[2] <= 0 or region[3] <= 0):
        raise ValueError("WIDTH i HEIGHT w --region muszą być większe od zera")
    if args.latency_ms < 0 or args.reaction_ms < 0:
        raise ValueError("Opóźnienia nie mogą być ujemne")

    cv2.namedWindow("Triki autopilot", cv2.WINDOW_NORMAL)
    waiting_frame = np.zeros((240, 420, 3), dtype=np.uint8)
    cv2.putText(
        waiting_frame,
        "BLE START: czekam na gre...",
        (18, 125),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (230, 230, 230),
        2,
        cv2.LINE_AA,
    )
    cv2.imshow("Triki autopilot", waiting_frame)

    delegate = PeripheralDelegate.alloc().init()
    manager = CBPeripheralManager.alloc().initWithDelegate_queue_options_(
        delegate, None, None
    )
    delegate.manager = manager

    vision = threading.Thread(
        target=vision_worker,
        args=(
            region,
            (args.latency_ms + args.reaction_ms) / 1000.0,
            control_queue,
            frame_queue,
        ),
        name="triki-vision",
        daemon=True,
    )
    vision.start()

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    LOG.info(
        "Bot działa; źródło=%s, predykcja=%.0f ms. q/Esc kończy program.",
        f"MSS {region}" if region is not None else "QuickTime Player (auto)",
        args.latency_ms + args.reaction_ms,
    )
    try:
        while not stop_event.is_set():
            CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.004, True)
            if delegate.ready_timer is None and manager.state() == CBManagerStatePoweredOn:
                delegate.start_timer()
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
                cv2.imshow("Triki autopilot", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                stop_event.set()
    finally:
        game_active.clear()
        stop_event.set()
        if delegate.ready_timer is not None:
            delegate.ready_timer.invalidate()
        manager.stopAdvertising()
        vision.join(timeout=2.0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
