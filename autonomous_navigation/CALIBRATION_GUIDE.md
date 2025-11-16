# Calibration Guide

Complete guide for Phase 1: System Integration & Calibration

## Overview

Before running autonomous navigation, you need to:
1. ✅ Verify red corner markers are detected
2. ✅ Calibrate world coordinate system
3. ✅ Test perspective transformation accuracy
4. ✅ Validate robot connection and sensors

## Quick Start

### Full Calibration (Recommended)

Run all calibration steps at once:

```bash
# 1. Capture calibration image from overhead camera
python3 autonomous_navigation/capture_calibration_image.py --url http://192.168.0.21:8000/

# 2. Run full calibration
python3 calibrate.py full-calibration --image calibration_image_*.png
```

### Step-by-Step Calibration

#### Step 1: Capture Calibration Image

Capture an image from the overhead camera showing the arena with red corner markers:

```bash
# From overhead camera stream
python3 autonomous_navigation/capture_calibration_image.py --url http://192.168.0.21:8000/

# Or from local webcam
python3 autonomous_navigation/capture_calibration_image.py --camera 0
```

**Requirements:**
- All 4 red corner markers must be clearly visible
- Good lighting conditions
- Arena should be in view
- Save the image for next steps

#### Step 2: Verify Red Corner Markers

Verify that all 4 red corner markers are detected correctly:

```bash
python3 calibrate.py verify-corners --image calibration_image_*.png
```

**Expected Output:**
- ✅ All 4 corners detected
- Corner coordinates displayed
- Visualization images saved (`red_corners_verified.png`, `red_mask_verified.png`)

**If Failed:**
- Check lighting conditions
- Ensure markers are not occluded
- Verify markers are red and clearly visible
- Try adjusting camera angle

#### Step 3: Calibrate World Coordinates

Calibrate the world coordinate system using measured arena dimensions:

```bash
# Using default coordinates (from arena_transform.py)
python3 calibrate.py calibrate-world --image calibration_image_*.png

# Using custom measured coordinates
python3 calibrate.py calibrate-world --image calibration_image_*.png \
    --world-corners "0,3.85 2.35,3.95 1.7,0.05 0.45,0"
```

**World Corner Format:**
- Order: Top-left, Top-right, Bottom-right, Bottom-left
- Units: Meters
- Format: "x1,y1 x2,y2 x3,y3 x4,y4"

**Expected Output:**
- ✅ Transformation matrices calculated
- Matrices saved to `auto_transform_matrix.npy` and `world_transform_matrix.npy`
- Test transformation displayed

**To Measure Coordinates:**
1. Measure distance from origin (bottom-left corner) to each corner
2. Record X and Y coordinates in meters
3. Use these values in `--world-corners`

#### Step 4: Test Perspective Transformation

Test the accuracy of the perspective transformation:

```bash
python3 calibrate.py test-transform --image calibration_image_*.png
```

**Expected Output:**
- ✅ Transformed image saved
- Corner angle analysis
- Quality assessment

**Quality Indicators:**
- Excellent: Angle error < 5°
- Good: Angle error < 10°
- Poor: Angle error > 10° (recalibrate)

#### Step 5: Validate Robot Connection

Test connection to the robot:

```bash
python3 calibrate.py validate-robot --robot-ip 192.168.1.216
```

**Expected Output:**
- ✅ Robot connection successful
- ✅ Communication test passed

**If Failed:**
- Verify robot IP address
- Check WiFi connection
- Ensure robot is powered on
- Verify WebSocket port 8765

#### Step 6: Validate Robot Sensors

Test all robot sensors:

```bash
python3 calibrate.py validate-sensors --robot-ip 192.168.1.216
```

**Expected Output:**
- ✅ Battery voltage readings
- ✅ Ultrasonic sensor readings (distance in cm)
- ✅ IR left sensor readings (0 or 1)
- ✅ IR right sensor readings (0 or 1)

**Testing Tips:**
- Move obstacles near sensors during test
- Verify sensors respond to obstacles
- Check battery voltage is adequate (>6V)

## Calibration Files

After successful calibration, these files are created:

- `auto_transform_matrix.npy` - Pixel to normalized coordinate transform
- `world_transform_matrix.npy` - Pixel to world coordinate transform
- `red_corners_verified.png` - Visualization of detected corners
- `red_mask_verified.png` - Red color detection mask

These files are used automatically by the navigation system.

## Troubleshooting

### Red Corners Not Detected

1. **Check lighting**: Ensure good, even lighting
2. **Verify marker color**: Markers must be clearly red
3. **Check camera angle**: All 4 corners must be visible
4. **Adjust HSV thresholds**: Modify in `arena_transform.py` if needed

### Transformation Quality Poor

1. **Recalibrate**: Run calibration again with better image
2. **Check measurements**: Verify world coordinates are accurate
3. **Camera position**: Ensure camera is stable and perpendicular

### Robot Connection Failed

1. **IP address**: Verify robot IP is correct
2. **Network**: Ensure robot and computer on same network
3. **Robot status**: Check robot is powered and connected to WiFi
4. **Firewall**: Check firewall isn't blocking port 8765

### Sensors Not Working

1. **Physical check**: Verify sensors are connected
2. **Obstacles**: Place obstacles near sensors to test
3. **Battery**: Low battery can affect sensor readings
4. **Firmware**: Ensure robot firmware is up to date

## Next Steps

After completing Phase 1 calibration:

1. ✅ Run simulation: `python3 scripts/simulate.py --stop-at-line 1`
2. ✅ Test on real robot: `python3 main.py --robot-ip 192.168.1.216 --camera-url http://192.168.0.21:8000/`
3. ✅ Monitor navigation and adjust parameters as needed

## Calibration Checklist

- [ ] Calibration image captured
- [ ] All 4 red corners detected
- [ ] World coordinates calibrated
- [ ] Transformation tested (angle error < 10°)
- [ ] Robot connection validated
- [ ] All sensors validated
- [ ] Calibration files saved

---

**Note**: Recalibrate if:
- Camera position changes
- Arena layout changes
- Transformation quality degrades
- Navigation accuracy decreases

