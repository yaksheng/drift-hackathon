"""Autonomous Galaxy RVR edge-follow navigator.

This module combines the onboard servo camera, OpenCV-based vision, and the
GalaxyRVR websocket API to keep the robot on the white mat, hug a selected
edge, and stop on a user-selected black strip.

High level behaviour
--------------------
1. Pitch the camera about 45° downward and continuously detect the boundary
   between the white mat and the darker surroundings.
2. Hug either the left or the right edge (configurable) while moving as fast
   as the vision loop allows.
3. If the edge is lost, immediately stop and yaw in place until the boundary
   re-enters the field of view.
4. When the black strip zone is detected, tilt the camera straight down and
   roll forward slowly, counting each strip until the requested one is
   reached.

The script favours responsiveness and configurability so it can be tuned at
competition time without code changes.
"""

from __future__ import annotations

import argparse
import asyncio
import time
from dataclasses import dataclass
from typing import List

import cv2
import numpy as np
import sys
import os

# Add robot_code/python_client to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'robot_code', 'python_client'))
from galaxyrvr import GalaxyRVR
from galaxyrvr_camera import CameraStream


@dataclass
class EdgeMeasurement:
    """Container for edge detection results."""

    positions: List[int]
    mask: np.ndarray
    roi_y_offset: int

    @property
    def found(self) -> bool:
        return len(self.positions) >= 2

    @property
    def average(self) -> float:
        return float(np.mean(self.positions)) if self.positions else -1.0

    @property
    def orientation_delta(self) -> float:
        if len(self.positions) < 2:
            return 0.0
        # rows are gathered from bottom to top, so index 0 is closest to robot
        return float(self.positions[0] - self.positions[-1])


class EdgeFollowNavigator:
    """Stateful controller that follows the mat edge and stops on a strip."""

    def __init__(
        self,
        robot: GalaxyRVR,
        camera: CameraStream,
        strip_target: int,
        hug_side: str = "right",
        base_speed: int = 70,
        edge_servo_angle: int = 135,
        down_servo_angle: int = 170,
    ) -> None:
        if strip_target not in (1, 2, 3):
            raise ValueError("strip_target must be 1, 2 or 3")
        if hug_side not in ("left", "right"):
            raise ValueError("hug_side must be 'left' or 'right'")

        self.robot = robot
        self.camera = camera
        self.strip_target = strip_target
        self.hug_side = hug_side
        self.base_speed = base_speed
        self.edge_servo_angle = edge_servo_angle
        self.down_servo_angle = down_servo_angle

        self.loop_delay = 0.03  # ~33 Hz vision loop
        self.edge_offset_ratio = 0.08
        self.lateral_gain = 0.035
        self.orientation_gain = 0.02
        self.max_turn = 40
        self.search_turn_speed = 35
        self.edge_lost_frames = 0
        self.edge_required_frames = 4
        self.searching = False
        self.strip_detect_frames = 0
        self.strip_detect_threshold = 6
        self.strip_coverage_threshold = 0.22
        self.current_strip = 0
        self.on_strip = False
        self.completed = False
        self.last_command_time = 0.0
        self.target_strip_detected = False  # Track if we've detected the target strip
        self.frames_after_target = 0  # Count frames after target strip detected
        self.max_frames_after_target = 30  # Stop after this many frames on target strip

    async def run(self) -> None:
        """Main control loop."""

        # Tilt camera for edge following
        self.robot.set_servo(self.edge_servo_angle)
        await self.robot.send()

        print(
            f"Hugging the {self.hug_side} edge and targeting strip {self.strip_target}."
        )

        try:
            while not self.completed and self.robot.running:
                frame = self.camera.get_frame()
                if frame is None:
                    await asyncio.sleep(self.loop_delay)
                    continue

                if self.searching:
                    measurement = self._find_edge(frame)
                    if measurement.found:
                        self.edge_lost_frames += 1
                        if self.edge_lost_frames >= self.edge_required_frames:
                            self.searching = False
                            self.edge_lost_frames = 0
                            print("Edge reacquired. Resuming forward motion.")
                    else:
                        self.edge_lost_frames = 0

                    self._spin_in_place()
                    await asyncio.sleep(self.loop_delay)
                    continue

                # Edge following phase
                measurement = self._find_edge(frame)
                if not measurement.found:
                    self.edge_lost_frames += 1
                    if self.edge_lost_frames > self.edge_required_frames:
                        await self._enter_search_mode()
                    else:
                        # Hold current pose briefly
                        self.robot.stop()
                        await self.robot.send()
                    await asyncio.sleep(self.loop_delay)
                    continue

                self.edge_lost_frames = 0
                self._follow_edge(measurement)

                # Check for strip zone using the angled camera
                if self._is_strip_zone_visible(frame):
                    self.strip_detect_frames += 1
                    if self.strip_detect_frames >= self.strip_detect_threshold:
                        print("Black strip zone detected. Switching to strip seek mode.")
                        await self._enter_strip_seek()
                else:
                    self.strip_detect_frames = max(0, self.strip_detect_frames - 1)

                await asyncio.sleep(self.loop_delay)
        except KeyboardInterrupt:
            print("Interrupted by user. Stopping robot...")
        finally:
            self.robot.stop()
            await self.robot.send()
            self.completed = True

    def _follow_edge(self, measurement: EdgeMeasurement) -> None:
        """Compute motor commands from the detected edge."""

        roi_width = measurement.mask.shape[1]
        if self.hug_side == "right":
            desired_position = roi_width * (1.0 - self.edge_offset_ratio)
        else:
            desired_position = roi_width * self.edge_offset_ratio

        lateral_error = desired_position - measurement.average
        orientation_error = measurement.orientation_delta
        turn = (self.lateral_gain * lateral_error) + (
            self.orientation_gain * orientation_error
        )
        turn = float(np.clip(turn, -self.max_turn, self.max_turn))

        left = int(np.clip(self.base_speed - turn, -100, 100))
        right = int(np.clip(self.base_speed + turn, -100, 100))
        self.robot.set_motors(left, right)
        self.robot.set_servo(self.edge_servo_angle)
        self._send_if_needed()

    def _spin_in_place(self) -> None:
        """Keep yawing while the edge is missing."""

        if self.hug_side == "right":
            self.robot.set_motors(self.search_turn_speed, -self.search_turn_speed)
        else:
            self.robot.set_motors(-self.search_turn_speed, self.search_turn_speed)
        self._send_if_needed()

    async def _enter_search_mode(self) -> None:
        """Stop and begin a yaw search for the lost edge."""

        print("Edge lost. Entering search mode...")
        self.robot.stop()
        await self.robot.send()
        self.searching = True
        self.edge_lost_frames = 0

    async def _enter_strip_seek(self) -> None:
        """Pitch the camera down and begin counting black strips."""

        self.robot.stop()
        await self.robot.send()
        await asyncio.sleep(0.2)
        self.robot.set_servo(self.down_servo_angle)
        await self.robot.send()
        self.current_strip = 0
        self.on_strip = False

        # Slow forward crawl to improve counting accuracy
        crawl_speed = 35
        print(
            "Camera pitched down. Crawling forward to count black strips..."
        )

        while not self.completed and self.robot.running:
            frame = self.camera.get_frame()
            if frame is None:
                await asyncio.sleep(self.loop_delay)
                continue

            coverage = self._black_coverage(frame)
            
            # Check if we're moving out of bounds (too much black = off the mat)
            if coverage > 0.7:  # Very high black coverage = likely off mat
                print("Warning: High black coverage detected. May be out of bounds. Stopping.")
                self.robot.stop()
                await self.robot.send()
                self.completed = True
                return
            
            if coverage > self.strip_coverage_threshold:
                if not self.on_strip:
                    self.on_strip = True
                    self.current_strip += 1
                    print(f"Detected strip {self.current_strip}.")
                    
                    # Check if this is the target strip - stop immediately when we enter it
                    if self.current_strip == self.strip_target:
                        print(f"Target strip {self.strip_target} reached! Stopping robot.")
                        self.robot.stop()
                        await self.robot.send()
                        await asyncio.sleep(0.1)  # Brief pause to ensure stop command is sent
                        self.completed = True
                        return
                    
                    # If we've passed the target, stop immediately
                    if self.current_strip > self.strip_target:
                        print(f"Passed target strip {self.strip_target}. Stopping robot.")
                        self.robot.stop()
                        await self.robot.send()
                        self.completed = True
                        return
                else:
                    # Still on strip - if this is the target, count frames to prevent overshoot
                    if self.current_strip == self.strip_target:
                        self.frames_after_target += 1
                        # Stop after a short time on the target strip to prevent moving past it
                        if self.frames_after_target > self.max_frames_after_target:
                            print(f"Stopped on target strip {self.strip_target}.")
                            self.robot.stop()
                            await self.robot.send()
                            self.completed = True
                            return
            else:
                if self.on_strip:
                    self.on_strip = False
                    print(f"Completed crossing strip {self.current_strip}.")
                    
                    # Safety check: if we've crossed more strips than target, stop immediately
                    if self.current_strip > self.strip_target:
                        print(f"Crossed {self.current_strip} strips, target was {self.strip_target}. Stopping.")
                        self.robot.stop()
                        await self.robot.send()
                        self.completed = True
                        return

            # Only continue crawling if we haven't reached the target yet
            if self.current_strip < self.strip_target:
                self.robot.set_motors(crawl_speed, crawl_speed)
            else:
                # We've reached or passed the target - stop moving
                self.robot.stop()
            
            self._send_if_needed()
            await asyncio.sleep(self.loop_delay)

    def _send_if_needed(self) -> None:
        """Rate-limit websocket sends to avoid saturating the ESP32."""

        now = time.perf_counter()
        if now - self.last_command_time >= 0.03:
            asyncio.create_task(self.robot.send())
            self.last_command_time = now

    def _find_edge(self, frame: np.ndarray) -> EdgeMeasurement:
        """Return the detected edge column positions."""

        h, w = frame.shape[:2]
        roi_start = int(h * 0.4)
        roi = frame[roi_start:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 180], dtype=np.uint8)
        upper_white = np.array([179, 60, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        rows = np.linspace(mask.shape[0] - 1, 0, num=6, dtype=int)
        positions: List[int] = []
        for r in rows:
            row = mask[r]
            white_pixels = np.where(row > 0)[0]
            if white_pixels.size == 0:
                continue
            if self.hug_side == "right":
                edge_x = int(white_pixels.max())
            else:
                edge_x = int(white_pixels.min())
            positions.append(edge_x)

        return EdgeMeasurement(positions=positions, mask=mask, roi_y_offset=roi_start)

    def _is_strip_zone_visible(self, frame: np.ndarray) -> bool:
        """Detect dark regions ahead with the angled camera."""

        coverage = self._black_coverage(frame)
        return coverage > (self.strip_coverage_threshold * 0.6)

    def _black_coverage(self, frame: np.ndarray) -> float:
        """Percentage of black pixels in the near-field region of the frame."""

        h, _ = frame.shape[:2]
        roi = frame[int(h * 0.55) :, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.medianBlur(mask, 5)
        coverage = float(np.mean(mask) / 255.0)
        return coverage


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Galaxy RVR edge-following navigator")
    parser.add_argument("--robot-ip", default="192.168.1.216", help="Galaxy RVR IP")
    parser.add_argument("--port", type=int, default=8765, help="Websocket port")
    parser.add_argument(
        "--hug-side",
        choices=["left", "right"],
        default="right",
        help="Which mat edge to hug",
    )
    parser.add_argument(
        "--strip", type=int, default=1, help="Target strip number (1=near, 3=far)"
    )
    parser.add_argument(
        "--speed", type=int, default=70, help="Base speed used during edge follow"
    )
    parser.add_argument(
        "--edge-servo-angle",
        type=int,
        default=135,
        help="Servo angle used while edge following (≈45° down)",
    )
    parser.add_argument(
        "--down-servo-angle",
        type=int,
        default=170,
        help="Servo angle when looking straight down at the strips",
    )
    return parser.parse_args()


async def main() -> None:
    args = parse_args()

    robot = GalaxyRVR(args.robot_ip, args.port)
    if not await robot.connect():
        print("Unable to connect to robot. Exiting...")
        return

    camera = CameraStream(args.robot_ip, display=False)
    camera.start()
    await asyncio.sleep(2.0)  # allow the camera stream to stabilise

    controller = EdgeFollowNavigator(
        robot=robot,
        camera=camera,
        strip_target=args.strip,
        hug_side=args.hug_side,
        base_speed=args.speed,
        edge_servo_angle=args.edge_servo_angle,
        down_servo_angle=args.down_servo_angle,
    )

    await controller.run()

    camera.stop()
    await robot.disconnect()
    print("Navigation routine finished.")


if __name__ == "__main__":
    asyncio.run(main())