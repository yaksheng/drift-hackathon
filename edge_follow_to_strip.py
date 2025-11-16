"""Edge following and strip selection routine for the Galaxy RVR robot."""
from __future__ import annotations

import argparse
import asyncio
import time
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np

from galaxyrvr import GalaxyRVR
from galaxyrvr_camera import CameraStream


@dataclass
class ControlConfig:
    """Tunable parameters for the controller."""

    approach_speed: int = 35
    follow_speed: int = 40
    strip_speed: int = 30
    turn_gain: float = 120.0
    camera_look_angle: int = 135  # ~45 degrees down
    camera_down_angle: int = 170  # nearly straight down
    desired_edge_fraction: float = 0.25
    desired_strip_row_fraction: float = 0.72
    edge_detection_threshold: float = 0.6
    strip_detection_ratio: float = 0.04
    strip_row_ratio_threshold: float = 0.08


class EdgeFollowerController:
    """High level controller for mat-edge following and strip alignment."""

    def __init__(self, robot: GalaxyRVR, camera: CameraStream, target_strip: int, config: ControlConfig):
        self.robot = robot
        self.camera = camera
        self.target_strip = target_strip
        self.config = config
        self._last_send = 0.0

    async def run(self) -> None:
        """Execute the full mission sequence."""
        print("Starting edge following mission…")
        self.robot.set_servo(self.config.camera_look_angle)
        await self.robot.send()

        await self._seek_white_edge()
        await self._follow_edge_until_strips()

        print("Switching to strip alignment phase…")
        self.robot.set_servo(self.config.camera_down_angle)
        await self.robot.send()
        await self._align_with_target_strip()

        print("Target strip reached. Mission complete!")

    async def _seek_white_edge(self) -> None:
        print("Phase 1: seeking the boundary of the white mat…")
        while True:
            frame = self.camera.get_frame()
            if frame is None:
                await asyncio.sleep(0.05)
                continue

            edge_ratio = self._white_ratio_near_horizon(frame)
            if edge_ratio < self.config.edge_detection_threshold:
                self.robot.stop()
                await self._throttled_send()
                print("Edge detected — transitioning to edge following phase.")
                return

            self.robot.forward(self.config.approach_speed)
            await self._throttled_send()
            await asyncio.sleep(0.05)

    async def _follow_edge_until_strips(self) -> None:
        print("Phase 2: following the mat edge while staying on the white surface…")
        strip_counter = 0
        while True:
            frame = self.camera.get_frame()
            if frame is None:
                await asyncio.sleep(0.05)
                continue

            error = self._edge_follow_error(frame)
            if error is None:
                self.robot.forward(self.config.follow_speed)
            else:
                correction = int(self.config.turn_gain * error)
                left = self.config.follow_speed - correction
                right = self.config.follow_speed + correction
                self.robot.set_motors(left, right)

            if self._black_strip_in_view(frame):
                strip_counter += 1
                if strip_counter > 5:
                    self.robot.stop()
                    await self._throttled_send()
                    print("Detected the strip region — preparing to align.")
                    return
            else:
                strip_counter = 0

            await self._throttled_send()
            await asyncio.sleep(0.05)

    async def _align_with_target_strip(self) -> None:
        print(f"Phase 3: searching for strip #{self.target_strip}…")
        desired_row = None

        while True:
            frame = self.camera.get_frame()
            if frame is None:
                await asyncio.sleep(0.05)
                continue

            if desired_row is None:
                desired_row = int(frame.shape[0] * self.config.desired_strip_row_fraction)

            strip_rows = self._locate_strip_rows(frame)
            if len(strip_rows) < self.target_strip:
                self.robot.forward(self.config.strip_speed)
                await self._throttled_send()
                await asyncio.sleep(0.05)
                continue

            target_row = strip_rows[self.target_strip - 1]
            error = desired_row - target_row

            if abs(error) < 12:
                self.robot.stop()
                await self._throttled_send()
                return

            speed = max(20, min(self.config.strip_speed + int(abs(error) * 0.1), 60))
            if error > 0:
                self.robot.forward(speed)
            else:
                self.robot.backward(speed)

            await self._throttled_send()
            await asyncio.sleep(0.05)

    def _white_ratio_near_horizon(self, frame: np.ndarray) -> float:
        mask = self._white_mask(frame)
        h = mask.shape[0]
        roi = mask[int(h * 0.45): int(h * 0.75), :]
        return float(cv2.countNonZero(roi)) / float(roi.size)

    def _edge_follow_error(self, frame: np.ndarray) -> Optional[float]:
        mask = self._white_mask(frame)
        h, w = mask.shape
        roi = mask[int(h * 0.35):, :]
        column_counts = np.sum(roi > 0, axis=0)
        min_pixels = roi.shape[0] * 0.25

        left_edge = next((idx for idx, val in enumerate(column_counts) if val > min_pixels), None)
        right_edge = next((w - 1 - idx for idx, val in enumerate(column_counts[::-1]) if val > min_pixels), None)

        if left_edge is None or right_edge is None:
            return None

        # Decide which edge to track based on proximity to frame borders
        if left_edge < (w - right_edge):
            edge_column = left_edge
            desired = int(w * self.config.desired_edge_fraction)
        else:
            edge_column = right_edge
            desired = int(w * (1 - self.config.desired_edge_fraction))

        return float(desired - edge_column) / float(w)

    def _black_strip_in_view(self, frame: np.ndarray) -> bool:
        mask = self._black_mask(frame)
        h = mask.shape[0]
        roi = mask[int(h * 0.25): int(h * 0.55), :]
        ratio = float(cv2.countNonZero(roi)) / float(roi.size)
        return ratio > self.config.strip_detection_ratio

    def _locate_strip_rows(self, frame: np.ndarray) -> List[int]:
        mask = self._black_mask(frame)
        h = mask.shape[0]
        row_density = np.mean(mask > 0, axis=1)
        threshold = self.config.strip_row_ratio_threshold

        rows: List[int] = []
        in_band = False
        band_start = band_end = 0

        for row in range(h - 1, -1, -1):
            if row_density[row] > threshold:
                if not in_band:
                    in_band = True
                    band_start = row
                band_end = row
            elif in_band:
                rows.append(int((band_start + band_end) / 2))
                in_band = False

        if in_band:
            rows.append(int((band_start + band_end) / 2))

        return rows

    def _white_mask(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 140])
        upper = np.array([180, 60, 255])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        return mask

    def _black_mask(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 0])
        upper = np.array([180, 255, 70])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.medianBlur(mask, 5)
        return mask

    async def _throttled_send(self) -> None:
        now = time.time()
        if now - self._last_send > 0.03:
            await self.robot.send()
            self._last_send = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Edge following routine for the Galaxy RVR robot.")
    parser.add_argument("strip", type=int, choices=(1, 2, 3), help="Target strip number (1=nearest, 3=furthest).")
    parser.add_argument("--robot-ip", default="192.168.1.216", help="IP address of the robot controller.")
    parser.add_argument("--port", type=int, default=8765, help="Robot websocket port (default: 8765).")
    parser.add_argument("--camera-port", type=int, default=9000, help="Camera streaming port (default: 9000).")
    parser.add_argument("--display", action="store_true", help="Display the onboard camera feed for debugging.")
    return parser.parse_args()


async def main() -> None:
    args = parse_args()
    robot = GalaxyRVR(args.robot_ip, args.port)
    if not await robot.connect():
        return

    camera = CameraStream(args.robot_ip, args.camera_port, display=args.display)
    camera.start()

    controller = EdgeFollowerController(robot, camera, args.strip, ControlConfig())

    try:
        await controller.run()
    except KeyboardInterrupt:
        print("Interrupted — stopping robot.")
    finally:
        robot.stop()
        await robot.send()
        await robot.disconnect()
        camera.stop()


if __name__ == "__main__":
    asyncio.run(main())