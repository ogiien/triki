#!/usr/bin/env python3
"""BLE peripheral and OpenCV autopilot for a QuickTime-mirrored game on macOS."""

# Protocol behavior adapted from Maku-hub/TrikiEmu, MIT license; see TrikiEmu/LICENSE.

from __future__ import annotations

import argparse
import logging
import signal
import struct
import threading
import time
from typing import Any

import cv2
import mss
import numpy as np
import objc
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
ACCEL_Z = 4096

stop_event = threading.Event()
steering_lock = threading.Lock()
accel_x = 0


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
        self.battery_characteristic = None
        self.firmware_characteristic = None
        self.ready_timer = None
        self.tx_subscribed = False
        self.battery_subscribed = False
        self.is_streaming = False
        self.ready_pending = False
        self.stream_buffer = bytearray()
        self.pending_chunks: list[bytes] = []
        self.services_pending = 0
        self.last_battery_update = time.monotonic()
        self.battery_update_pending = False
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
                NSData.dataWithBytes_length_(b"\x00", 1),
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
                NSData.dataWithBytes_length_(b"\x64", 1),
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
        if error is not None:
            LOG.error("Nie udało się dodać usługi BLE: %s", error)
            stop_event.set()
            return
        self.services_pending -= 1
        if self.services_pending == 0:
            # CoreBluetooth has no API for manufacturer data or a separate scan response.
            LOG.warning(
                "CoreBluetooth nie ustawia MAC i nie emuluje manufacturer data ani osobnego scan response"
            )
            self.advertisement_data = {
                CBAdvertisementDataLocalNameKey: DEVICE_NAME,
                CBAdvertisementDataServiceUUIDsKey: [
                    CBUUID.UUIDWithString_(SERVICE_UUID),
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
                    LOG.info("RX: START")
                elif value[:2] == b"\x20\x00":
                    self.is_streaming = False
                    self.stream_buffer.clear()
                    self.pending_chunks.clear()
                    LOG.info("RX: STOP")
            elif uuid == VENDOR_UUID and value:
                masked_led = bytes((value[0] & 0x01,))
                self.vendor_characteristic.setValue_(
                    NSData.dataWithBytes_length_(masked_led, 1)
                )
        if requests:
            manager.respondToRequest_withResult_(requests[0], CBATTErrorSuccess)

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

        now = time.monotonic()
        if self.battery_subscribed and now - self.last_battery_update >= 5.0:
            self.last_battery_update = now
            self.battery_update_pending = True
        if self.battery_update_pending and self.battery_subscribed:
            battery = NSData.dataWithBytes_length_(b"\x64", 1)
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
            with steering_lock:
                current_accel_x = accel_x
            self.stream_buffer.extend(make_imu_frame(current_accel_x))
            if len(self.stream_buffer) == 42:
                self.pending_chunks = [
                    bytes(self.stream_buffer[:size])
                    for size in STREAM_NOTIFY_SIZES
                ]
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


def vision_worker(region: tuple[int, int, int, int] | None) -> None:
    global accel_x

    cv2.namedWindow("Triki autopilot", cv2.WINDOW_NORMAL)
    try:
        with mss.mss() as screen:
            if region is None:
                if len(screen.monitors) < 2:
                    raise RuntimeError("Nie znaleziono ekranu do przechwycenia")
                monitor = dict(screen.monitors[1])
                LOG.info("Przechwytuję ekran główny; ustaw --region dla kadru QuickTime")
            else:
                left, top, width, height = region
                monitor = {"left": left, "top": top, "width": width, "height": height}
                LOG.info("Przechwytuję region QuickTime: %s", monitor)

            while not stop_event.is_set():
                raw = np.asarray(screen.grab(monitor))
                frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                height, width = frame.shape[:2]
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
                    np.ones((3, 3), dtype=np.uint8),
                )
                clouds = _detect_clouds(
                    purple_mask, max(50.0, width * height * 0.00006)
                )

                lookahead = int(height * 0.28)
                margin = max(10, int(width * 0.018))
                threats = [
                    cloud
                    for cloud in clouds
                    if cloud[1] < ship_y
                    and cloud[1] + cloud[3] >= ship_y - lookahead
                    and cloud[0] < center_x + ship_half_width
                    and cloud[0] + cloud[2] > center_x - ship_half_width
                ]
                nearest = max(threats, key=lambda item: item[1] + item[3], default=None)

                target_accel = 0
                direction = "LOT PROSTO"
                if nearest is not None:
                    cloud_x, cloud_y, cloud_width, cloud_height = nearest
                    free_left = max(0, cloud_x - margin)
                    free_right = max(0, width - (cloud_x + cloud_width) - margin)
                    if free_left >= free_right:
                        target_accel = -12000
                        direction = "SKRĘT W LEWO"
                    else:
                        target_accel = 12000
                        direction = "SKRĘT W PRAWO"
                    cv2.arrowedLine(
                        frame,
                        (center_x, ship_y),
                        (center_x + (-width // 5 if target_accel < 0 else width // 5), ship_y - 70),
                        (0, 255, 255),
                        3,
                        tipLength=0.18,
                    )

                # Smooth the return to level flight and limit abrupt steering changes.
                with steering_lock:
                    accel_x = int(accel_x + (target_accel - accel_x) * 0.18)
                    shown_accel = accel_x

                for cloud in clouds:
                    is_nearest = cloud is nearest
                    x, y, cloud_width, cloud_height = cloud
                    cv2.rectangle(
                        frame,
                        (x, y),
                        (x + cloud_width, y + cloud_height),
                        (255, 0, 255) if is_nearest else (180, 80, 210),
                        2 if is_nearest else 1,
                    )

                ship_box = (center_x - ship_half_width, ship_y - 12,
                            center_x + ship_half_width, ship_y + 12)
                cv2.rectangle(frame, ship_box[:2], ship_box[2:], (40, 220, 80), 2)
                cv2.line(frame, (center_x, 0), (center_x, height), (90, 90, 90), 1)
                cv2.putText(
                    frame,
                    f"{direction} | accel_x={shown_accel} | chmury={len(clouds)}",
                    (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Triki autopilot", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    stop_event.set()
                    break
                time.sleep(1 / 30)
    except Exception:
        LOG.exception("Wątek wizji zakończył się błędem")
        stop_event.set()
    finally:
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--region",
        nargs=4,
        type=int,
        metavar=("LEFT", "TOP", "WIDTH", "HEIGHT"),
        help="Wycinek ekranu zawierający okno QuickTime; bez opcji analizowany jest ekran główny",
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

    delegate = PeripheralDelegate.alloc().init()
    manager = CBPeripheralManager.alloc().initWithDelegate_queue_options_(
        delegate, None, None
    )
    delegate.manager = manager

    vision = threading.Thread(
        target=vision_worker,
        args=(region,),
        name="triki-vision",
        daemon=True,
    )
    vision.start()

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    LOG.info("Bot działa. q/Esc w podglądzie lub Ctrl+C kończy program.")
    try:
        while not stop_event.is_set():
            CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, True)
            if delegate.ready_timer is None and manager.state() == CBManagerStatePoweredOn:
                delegate.start_timer()
    finally:
        if delegate.ready_timer is not None:
            delegate.ready_timer.invalidate()
        manager.stopAdvertising()
        stop_event.set()
        vision.join(timeout=2.0)


if __name__ == "__main__":
    main()
