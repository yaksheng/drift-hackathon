#!/usr/bin/env python3
"""
Capture Calibration Image

Captures an image from the overhead camera for calibration purposes.
"""

import cv2
import requests
import numpy as np
import argparse
import sys
import os
from datetime import datetime


def capture_from_url(url: str, output_path: str = None) -> bool:
    """
    Capture image from camera stream URL
    
    Args:
        url: Camera stream URL (e.g., http://192.168.0.21:8000/)
        output_path: Path to save image (default: timestamped filename)
        
    Returns:
        True if successful
    """
    if output_path is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f'calibration_image_{timestamp}.png'
    
    print(f"📸 Capturing image from {url}...")
    print(f"   Saving to: {output_path}")
    
    try:
        # Get stream
        stream = requests.get(url, stream=True, timeout=10)
        
        if stream.status_code != 200:
            print(f"❌ Error: Server returned status code {stream.status_code}")
            return False
        
        # Parse MJPEG stream
        bytes_data = bytes()
        for chunk in stream.iter_content(chunk_size=1024):
            bytes_data += chunk
            
            # Find JPEG boundaries
            a = bytes_data.find(b'\xff\xd8')
            b = bytes_data.find(b'\xff\xd9')
            
            if a != -1 and b != -1:
                jpg = bytes_data[a:b+2]
                
                # Decode image
                image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                if image is not None:
                    # Save image
                    cv2.imwrite(output_path, image)
                    print(f"✅ Image captured: {image.shape[1]}x{image.shape[0]} pixels")
                    print(f"✅ Saved to: {output_path}")
                    return True
        
        print("❌ Error: Could not decode image from stream")
        return False
        
    except requests.exceptions.RequestException as e:
        print(f"❌ Connection error: {e}")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def capture_from_webcam(camera_index: int = 0, output_path: str = None) -> bool:
    """
    Capture image from local webcam
    
    Args:
        camera_index: Camera index (default: 0)
        output_path: Path to save image
        
    Returns:
        True if successful
    """
    if output_path is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f'calibration_image_{timestamp}.png'
    
    print(f"📸 Capturing from webcam {camera_index}...")
    print(f"   Saving to: {output_path}")
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"❌ Error: Could not open camera {camera_index}")
        return False
    
    # Capture frame
    ret, frame = cap.read()
    cap.release()
    
    if not ret or frame is None:
        print("❌ Error: Could not capture frame")
        return False
    
    # Save image
    cv2.imwrite(output_path, frame)
    print(f"✅ Image captured: {frame.shape[1]}x{frame.shape[0]} pixels")
    print(f"✅ Saved to: {output_path}")
    return True


def main():
    parser = argparse.ArgumentParser(description='Capture calibration image')
    parser.add_argument('--url', type=str, default=None,
                       help='Camera stream URL (e.g., http://192.168.0.21:8000/)')
    parser.add_argument('--camera', type=int, default=None,
                       help='Local webcam index (e.g., 0, 1, 2)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output image path (default: timestamped)')
    
    args = parser.parse_args()
    
    if args.url:
        success = capture_from_url(args.url, args.output)
    elif args.camera is not None:
        success = capture_from_webcam(args.camera, args.output)
    else:
        # Try URL first, then webcam
        default_url = "http://192.168.0.21:8000/"
        print(f"⚠️  No source specified, trying URL: {default_url}")
        success = capture_from_url(default_url, args.output)
        
        if not success:
            print("\n⚠️  URL capture failed, trying local webcam...")
            success = capture_from_webcam(0, args.output)
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

