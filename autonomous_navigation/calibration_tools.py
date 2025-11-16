"""
Calibration Tools

Tools for arena setup, calibration, and system validation.
Includes red corner marker verification, world coordinate calibration,
perspective transformation testing, and robot sensor validation.
"""

import cv2
import numpy as np
import sys
import os
from typing import Tuple, Optional, List
import argparse

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from webcam_code.arena_transform import detect_red_corners, transform_arena_auto


class ArenaCalibrator:
    """Tools for arena calibration and verification"""
    
    def __init__(self, camera_url: Optional[str] = None):
        """
        Initialize calibrator
        
        Args:
            camera_url: URL of overhead camera stream
        """
        self.camera_url = camera_url
    
    def verify_red_corners(self, image_path: str) -> Tuple[bool, List, np.ndarray]:
        """
        Verify red corner markers are detected correctly
        
        Args:
            image_path: Path to arena image
            
        Returns:
            (success, corners, mask) - success status, corner points, detection mask
        """
        print("=" * 60)
        print("Red Corner Marker Verification")
        print("=" * 60)
        
        # Read image
        image = cv2.imread(image_path)
        if image is None:
            print(f"❌ Error: Could not read image from {image_path}")
            return False, [], None
        
        print(f"\n📸 Image loaded: {image.shape[1]}x{image.shape[0]} pixels")
        
        # Detect corners
        corners, mask = detect_red_corners(image)
        
        if len(corners) == 4:
            print(f"\n✅ SUCCESS: All 4 red corner markers detected!")
            print("\nCorner coordinates (pixels):")
            labels = ["Top-left", "Top-right", "Bottom-right", "Bottom-left"]
            for i, (x, y) in enumerate(corners):
                print(f"  {labels[i]}: ({x:.0f}, {y:.0f})")
            
            # Visualize
            vis_image = image.copy()
            for i, (x, y) in enumerate(corners):
                cv2.circle(vis_image, (int(x), int(y)), 15, (0, 255, 0), -1)
                cv2.putText(vis_image, str(i+1), (int(x)+20, int(y)-20),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            
            # Draw quadrilateral
            pts = np.int32(corners)
            cv2.polylines(vis_image, [pts], True, (0, 255, 0), 3)
            
            cv2.imwrite('red_corners_verified.png', vis_image)
            cv2.imwrite('red_mask_verified.png', mask)
            print("\n✅ Visualization saved:")
            print("   - red_corners_verified.png")
            print("   - red_mask_verified.png")
            
            return True, corners, mask
        else:
            print(f"\n❌ FAILED: Only {len(corners)} corners detected (need 4)")
            print("\nTroubleshooting:")
            print("  1. Ensure all 4 red corner markers are clearly visible")
            print("  2. Check lighting conditions")
            print("  3. Verify red markers are not occluded")
            print("  4. Try adjusting camera angle")
            
            if len(corners) > 0:
                print(f"\nDetected {len(corners)} corners:")
                for i, (x, y) in enumerate(corners):
                    print(f"  Corner {i+1}: ({x:.0f}, {y:.0f})")
            
            return False, corners, mask
    
    def calibrate_world_coordinates(self,
                                   image_path: str,
                                   world_corners: List[Tuple[float, float]],
                                   output_width: int = 720,
                                   output_height: int = 1280) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Calibrate world coordinate system
        
        Args:
            image_path: Path to arena image
            world_corners: List of (x, y) world coordinates in meters
                          Order: [top-left, top-right, bottom-right, bottom-left]
            output_width: Output image width
            output_height: Output image height
            
        Returns:
            (success, transform_matrix)
        """
        print("=" * 60)
        print("World Coordinate System Calibration")
        print("=" * 60)
        
        if len(world_corners) != 4:
            print(f"❌ Error: Need exactly 4 world corner coordinates, got {len(world_corners)}")
            return False, None
        
        # Detect corners
        image = cv2.imread(image_path)
        if image is None:
            print(f"❌ Error: Could not read image from {image_path}")
            return False, None
        
        corners, _ = detect_red_corners(image)
        
        if len(corners) != 4:
            print(f"❌ Error: Could not detect 4 corners. Detected {len(corners)}")
            return False, None
        
        print(f"\n✅ Detected 4 corner markers")
        print("\nPixel coordinates → World coordinates:")
        labels = ["Top-left", "Top-right", "Bottom-right", "Bottom-left"]
        for i, ((px, py), (wx, wy)) in enumerate(zip(corners, world_corners)):
            print(f"  {labels[i]}: ({px:.0f}, {py:.0f}) px → ({wx:.2f}, {wy:.2f}) m")
        
        # Perform transformation
        src_points = np.float32(corners)
        dst_points = np.float32([
            [0, 0],
            [output_width, 0],
            [output_width, output_height],
            [0, output_height]
        ])
        
        # Calculate perspective transform
        transform_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        
        # Calculate world coordinate transform
        world_points = np.float32(world_corners)
        world_matrix, _ = cv2.findHomography(src_points, world_points)
        
        # Save matrices
        np.save('auto_transform_matrix.npy', transform_matrix)
        np.save('world_transform_matrix.npy', world_matrix)
        
        print("\n✅ Transformation matrices calculated and saved:")
        print("   - auto_transform_matrix.npy (pixel to normalized)")
        print("   - world_transform_matrix.npy (pixel to world coordinates)")
        
        # Test transformation
        test_point_px = np.array([image.shape[1]/2, image.shape[0]/2, 1.0])
        test_point_world = world_matrix @ test_point_px
        print(f"\n📊 Test transformation:")
        print(f"   Center pixel ({test_point_px[0]:.0f}, {test_point_px[1]:.0f})")
        print(f"   → World ({test_point_world[0]:.2f}, {test_point_world[1]:.2f}) m")
        
        return True, world_matrix
    
    def test_perspective_transform(self,
                                  image_path: str,
                                  output_path: str = 'transform_test.png') -> bool:
        """
        Test perspective transformation accuracy
        
        Args:
            image_path: Path to input image
            output_path: Path to save transformed image
            
        Returns:
            True if successful
        """
        print("=" * 60)
        print("Perspective Transformation Accuracy Test")
        print("=" * 60)
        
        # Check if transform matrix exists
        if not os.path.exists('auto_transform_matrix.npy'):
            print("❌ Error: Transform matrix not found!")
            print("   Run world coordinate calibration first.")
            return False
        
        transform_matrix = np.load('auto_transform_matrix.npy')
        print(f"\n✅ Loaded transformation matrix")
        
        # Read image
        image = cv2.imread(image_path)
        if image is None:
            print(f"❌ Error: Could not read image from {image_path}")
            return False
        
        # Detect corners
        corners, _ = detect_red_corners(image)
        if len(corners) != 4:
            print(f"❌ Error: Could not detect 4 corners")
            return False
        
        # Apply transformation
        output_width = 720
        output_height = 1280
        transformed = cv2.warpPerspective(image, transform_matrix, 
                                         (output_width, output_height))
        
        # Save result
        cv2.imwrite(output_path, transformed)
        print(f"\n✅ Transformed image saved: {output_path}")
        
        # Analyze transformation quality
        print("\n📊 Transformation Quality Analysis:")
        
        # Check if corners form a rectangle in transformed space
        src_points = np.float32(corners)
        dst_points = cv2.perspectiveTransform(src_points.reshape(-1, 1, 2), 
                                             transform_matrix).reshape(-1, 2)
        
        # Calculate angles (should be ~90 degrees for rectangle)
        def angle_between_points(p1, p2, p3):
            v1 = p2 - p1
            v2 = p3 - p2
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            return np.arccos(np.clip(cos_angle, -1, 1)) * 180 / np.pi
        
        angles = []
        for i in range(4):
            p1 = dst_points[i]
            p2 = dst_points[(i+1) % 4]
            p3 = dst_points[(i+2) % 4]
            angles.append(angle_between_points(p1, p2, p3))
        
        avg_angle = np.mean(angles)
        angle_error = abs(avg_angle - 90)
        
        print(f"   Average corner angle: {avg_angle:.1f}° (ideal: 90°)")
        print(f"   Angle error: {angle_error:.1f}°")
        
        if angle_error < 5:
            print("   ✅ Excellent transformation quality")
        elif angle_error < 10:
            print("   ⚠️  Good transformation quality")
        else:
            print("   ❌ Poor transformation quality - recalibrate")
        
        return True


class RobotValidator:
    """Tools for validating robot connection and sensors"""
    
    def __init__(self, robot_ip: str = "192.168.1.216"):
        """
        Initialize validator
        
        Args:
            robot_ip: IP address of robot
        """
        self.robot_ip = robot_ip
    
    async def validate_connection(self) -> bool:
        """
        Validate robot connection
        
        Returns:
            True if connection successful
        """
        print("=" * 60)
        print("Robot Connection Validation")
        print("=" * 60)
        
        try:
            import asyncio
            sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robot_code', 'python_client'))
            from galaxyrvr import GalaxyRVR
            
            print(f"\n🔌 Connecting to robot at {self.robot_ip}...")
            robot = GalaxyRVR(self.robot_ip)
            
            if await robot.connect():
                print("✅ Robot connection successful!")
                
                # Test communication
                robot.stop()
                await robot.send()
                await asyncio.sleep(0.5)
                
                print("✅ Communication test passed")
                
                await robot.disconnect()
                return True
            else:
                print("❌ Robot connection failed!")
                print("\nTroubleshooting:")
                print("  1. Verify robot IP address is correct")
                print("  2. Check WiFi connection")
                print("  3. Ensure robot is powered on")
                print("  4. Check WebSocket port 8765 is accessible")
                return False
                
        except Exception as e:
            print(f"❌ Error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    async def validate_sensors(self, duration: float = 5.0) -> bool:
        """
        Validate robot sensors
        
        Args:
            duration: How long to read sensors (seconds)
            
        Returns:
            True if sensors working
        """
        print("=" * 60)
        print("Robot Sensor Validation")
        print("=" * 60)
        
        try:
            import asyncio
            sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robot_code', 'python_client'))
            from galaxyrvr import GalaxyRVR
            
            print(f"\n🔌 Connecting to robot at {self.robot_ip}...")
            robot = GalaxyRVR(self.robot_ip)
            
            if not await robot.connect():
                print("❌ Connection failed!")
                return False
            
            print("✅ Connected")
            print(f"\n📊 Reading sensors for {duration} seconds...")
            print("   (Move obstacles near sensors to test)\n")
            
            readings = {
                'battery': [],
                'ultrasonic': [],
                'ir_left': [],
                'ir_right': []
            }
            
            start_time = asyncio.get_event_loop().time()
            while (asyncio.get_event_loop().time() - start_time) < duration:
                readings['battery'].append(robot.battery_voltage)
                readings['ultrasonic'].append(robot.ultrasonic_distance)
                readings['ir_left'].append(robot.ir_left)
                readings['ir_right'].append(robot.ir_right)
                
                print(f"\rBattery: {robot.battery_voltage:.2f}V | "
                      f"Ultrasonic: {robot.ultrasonic_distance}cm | "
                      f"IR Left: {robot.ir_left} | "
                      f"IR Right: {robot.ir_right}", end='', flush=True)
                
                await asyncio.sleep(0.1)
            
            print("\n\n📊 Sensor Validation Results:")
            
            # Battery
            if readings['battery']:
                avg_battery = np.mean([v for v in readings['battery'] if v is not None])
                print(f"  Battery: {avg_battery:.2f}V", end='')
                if avg_battery > 6.0:
                    print(" ✅")
                else:
                    print(" ⚠️  Low battery!")
            else:
                print("  Battery: ❌ No readings")
            
            # Ultrasonic
            ultrasonic_readings = [v for v in readings['ultrasonic'] if v is not None]
            if ultrasonic_readings:
                print(f"  Ultrasonic: {len(ultrasonic_readings)} readings, "
                      f"range: {min(ultrasonic_readings):.1f}-{max(ultrasonic_readings):.1f}cm ✅")
            else:
                print("  Ultrasonic: ❌ No readings")
            
            # IR sensors
            ir_left_readings = [v for v in readings['ir_left'] if v is not None]
            ir_right_readings = [v for v in readings['ir_right'] if v is not None]
            if ir_left_readings:
                print(f"  IR Left: {len(ir_left_readings)} readings, "
                      f"values: {set(ir_left_readings)} ✅")
            else:
                print("  IR Left: ❌ No readings")
            
            if ir_right_readings:
                print(f"  IR Right: {len(ir_right_readings)} readings, "
                      f"values: {set(ir_right_readings)} ✅")
            else:
                print("  IR Right: ❌ No readings")
            
            await robot.disconnect()
            
            # Overall validation
            all_working = (ultrasonic_readings and ir_left_readings and ir_right_readings)
            if all_working:
                print("\n✅ All sensors working correctly!")
                return True
            else:
                print("\n⚠️  Some sensors may not be working correctly")
                return False
                
        except Exception as e:
            print(f"\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """Main calibration tool"""
    parser = argparse.ArgumentParser(description='Arena Calibration and Validation Tools')
    parser.add_argument('command', choices=['verify-corners', 'calibrate-world', 
                                           'test-transform', 'validate-robot', 
                                           'validate-sensors', 'full-calibration'],
                       help='Calibration command to run')
    parser.add_argument('--image', type=str, default='test-real.png',
                       help='Path to arena image')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.216',
                       help='Robot IP address')
    parser.add_argument('--world-corners', type=str, default=None,
                       help='World corner coordinates as "x1,y1 x2,y2 x3,y3 x4,y4" in meters')
    
    args = parser.parse_args()
    
    if args.command == 'verify-corners':
        calibrator = ArenaCalibrator()
        success, corners, mask = calibrator.verify_red_corners(args.image)
        sys.exit(0 if success else 1)
    
    elif args.command == 'calibrate-world':
        if args.world_corners is None:
            # Use default coordinates from arena_transform.py
            world_corners = [
                (0, 3.85),      # Top-left
                (2.35, 3.95),   # Top-right
                (1.7, 0.05),    # Bottom-right
                (0.45, 0)       # Bottom-left
            ]
            print("⚠️  Using default world coordinates. Use --world-corners to specify custom.")
        else:
            # Parse world corners
            corners_str = args.world_corners.split()
            world_corners = []
            for corner_str in corners_str:
                x, y = map(float, corner_str.split(','))
                world_corners.append((x, y))
        
        calibrator = ArenaCalibrator()
        success, matrix = calibrator.calibrate_world_coordinates(args.image, world_corners)
        sys.exit(0 if success else 1)
    
    elif args.command == 'test-transform':
        calibrator = ArenaCalibrator()
        success = calibrator.test_perspective_transform(args.image)
        sys.exit(0 if success else 1)
    
    elif args.command == 'validate-robot':
        import asyncio
        validator = RobotValidator(args.robot_ip)
        success = asyncio.run(validator.validate_connection())
        sys.exit(0 if success else 1)
    
    elif args.command == 'validate-sensors':
        import asyncio
        validator = RobotValidator(args.robot_ip)
        success = asyncio.run(validator.validate_sensors())
        sys.exit(0 if success else 1)
    
    elif args.command == 'full-calibration':
        print("=" * 60)
        print("Full System Calibration")
        print("=" * 60)
        
        # Step 1: Verify corners
        print("\n[1/4] Verifying red corner markers...")
        calibrator = ArenaCalibrator()
        success, corners, _ = calibrator.verify_red_corners(args.image)
        if not success:
            print("\n❌ Corner verification failed. Cannot continue.")
            sys.exit(1)
        
        # Step 2: Calibrate world coordinates
        print("\n[2/4] Calibrating world coordinates...")
        if args.world_corners:
            corners_str = args.world_corners.split()
            world_corners = [tuple(map(float, c.split(','))) for c in corners_str]
        else:
            world_corners = [
                (0, 3.85), (2.35, 3.95), (1.7, 0.05), (0.45, 0)
            ]
        success, _ = calibrator.calibrate_world_coordinates(args.image, world_corners)
        if not success:
            print("\n❌ World coordinate calibration failed.")
            sys.exit(1)
        
        # Step 3: Test transformation
        print("\n[3/4] Testing perspective transformation...")
        success = calibrator.test_perspective_transform(args.image)
        if not success:
            print("\n⚠️  Transformation test had issues, but continuing...")
        
        # Step 4: Validate robot
        print("\n[4/4] Validating robot connection...")
        import asyncio
        validator = RobotValidator(args.robot_ip)
        success = asyncio.run(validator.validate_connection())
        if not success:
            print("\n⚠️  Robot validation failed, but calibration files are saved.")
        
        print("\n" + "=" * 60)
        print("✅ Full calibration complete!")
        print("=" * 60)
        print("\nNext steps:")
        print("  1. Run sensor validation: python3 calibration_tools.py validate-sensors")
        print("  2. Test navigation: python3 scripts/simulate.py --stop-at-line 1")


if __name__ == "__main__":
    main()

