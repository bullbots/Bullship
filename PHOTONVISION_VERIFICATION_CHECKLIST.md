# PhotonVision Dual-Camera Verification & Testing Checklist

## 📋 Pre-Deployment Verification

### 1. Physical Camera Measurements
**Location:** Update values in `Constants.java` → `VisionConstants`

#### Front Left Camera
- [ ] **X Position** (forward/back from robot center): Currently `12.056 inches`
  - Measure from robot center to camera
  - Positive = forward, Negative = backward

- [ ] **Y Position** (left/right from robot center): Currently `10.981 inches`
  - Measure from robot center to camera
  - Positive = left, Negative = right

- [ ] **Z Position** (up/down from robot center): Currently `8.44 inches`
  - Measure from robot center to camera lens
  - Positive = up, Negative = down

- [ ] **Pitch Angle** (up/down tilt): Currently `-24.094°`
  - Negative = angled down, Positive = angled up
  - Use protractor or level app on phone

- [ ] **Yaw Angle** (left/right rotation): Currently `30°`
  - Positive = angled left, Negative = angled right
  - Measure from robot's forward direction

#### Front Right Camera
- [ ] **X Position**: Currently `12.056 inches`
- [ ] **Y Position**: Currently `-10.981 inches` (right side)
- [ ] **Z Position**: Currently `8.44 inches`
- [ ] **Pitch Angle**: Currently `-24.094°`
- [ ] **Yaw Angle**: Currently `-30°` (angled right)

### 2. PhotonVision Configuration
**Access at:** http://10.18.91.11:5800

- [ ] PhotonVision is accessible at the static IP
- [ ] Both cameras appear in the camera list:
  - [ ] `FrontLeftCamera`
  - [ ] `FrontRightCamera`
- [ ] Camera names match exactly (case-sensitive):
  - Constants.java: `FRONT_LEFT_CAMERA_NAME` = "FrontLeftCamera"
  - Constants.java: `FRONT_RIGHT_CAMERA_NAME` = "FrontRightCamera"
- [ ] Both cameras are set to AprilTag pipeline
- [ ] Resolution is 640x480 for both cameras
- [ ] Frame rate is ~100 FPS for both cameras
- [ ] Cameras are calibrated (check calibration error < 1.0px is ideal, yours are 37.81px and 44.48px)
- [ ] 3D mode is available for both cameras

### 3. Network Configuration
- [ ] PhotonVision static IP: `10.18.91.11`
- [ ] roboRIO IP: `10.18.91.2`
- [ ] NetworkTables connection shows "Connected" in PhotonVision UI
- [ ] Team number is set to `1891`

---

## 🔧 Build & Deploy

### 4. Java Version Check
- [ ] Java 17 or newer is installed
  - Run: `java -version`
  - Should show "17" or higher, not "1.8"
- [ ] JAVA_HOME environment variable points to Java 17+

### 5. Code Compilation
- [ ] Run `./gradlew build` successfully
- [ ] No compilation errors
- [ ] No warnings about PhotonVision dependencies

### 6. Deploy to roboRIO
- [ ] Robot is powered on
- [ ] Computer is connected to robot network
- [ ] Run `./gradlew deploy`
- [ ] Deployment completes without errors
- [ ] Check Driver Station for no red errors

---

## 🧪 Testing Phase

### 7. Initial Connection Test
**Tools:** Shuffleboard, Glass, or AdvantageScope

- [ ] Open NetworkTables viewer (Shuffleboard/Glass)
- [ ] Verify PhotonVision tables exist:
  - [ ] `/photonvision/FrontLeftCamera`
  - [ ] `/photonvision/FrontRightCamera`
- [ ] Check that both cameras show activity (timestamps updating)

### 8. Single Camera Tests

#### Front Left Camera
- [ ] Point robot at a single AprilTag (2-4 meters away)
- [ ] Verify in NetworkTables:
  - [ ] `hasTargets` = true
  - [ ] `targetID` shows correct AprilTag number
  - [ ] `targetPitch` and `targetYaw` values seem reasonable
- [ ] Check Driver Station for latency warnings
- [ ] Verify pose estimation updates on Field2d widget

#### Front Right Camera
- [ ] Repeat single tag test for right camera
- [ ] Compare results with left camera (should be similar if viewing same tag)

### 9. Multi-Tag Tests
- [ ] Position robot to see 2+ AprilTags simultaneously
- [ ] Verify both cameras detect multiple tags
- [ ] Check that pose estimation uses multi-tag strategy
- [ ] Compare accuracy with single-tag detection (should be better)

### 10. Standard Deviation Tuning
**Location:** `Constants.java` → `VisionConstants`

Current values:
- `SINGLE_TAG_STD_DEVS = [4, 4, 8]`
- `MULTI_TAG_STD_DEVS = [0.5, 0.5, 1]`

Testing procedure:
- [ ] Drive robot in a square pattern near AprilTags
- [ ] Log odometry vs vision-corrected pose
- [ ] If vision corrections are too aggressive (jerky movement):
  - Increase values (trust vision less)
- [ ] If odometry drifts too much:
  - Decrease values (trust vision more)
- [ ] Test at different distances (1m, 2m, 4m, 6m)

### 11. Distance-Based Filtering
- [ ] Test at 1 meter from tag - should accept readings
- [ ] Test at 4 meters from tag - check if still accurate
- [ ] Test at 6+ meters - readings might be rejected (check line 570-571 in Vision.java)
- [ ] Verify single tag readings beyond 4m are rejected

### 12. Ambiguity Filtering
- [ ] Check `poseAmbiguity` values in NetworkTables
- [ ] Values should be < 0.25 for good detections
- [ ] High ambiguity (> 0.25) should be filtered out

### 13. Latency Monitoring
- [ ] Watch Driver Station for Alert messages
- [ ] If "high latency" warnings appear:
  - Check PhotonVision CPU usage
  - Verify network connection quality
  - Consider reducing camera resolution or FPS

### 14. Autonomous Testing
- [ ] Run a PathPlanner autonomous routine
- [ ] Verify robot follows path accurately
- [ ] Check if vision corrections keep robot on track
- [ ] Test with and without AprilTags visible

### 15. Field2d Visualization
In Shuffleboard/Glass:
- [ ] Robot pose updates smoothly
- [ ] "VisionEstimation" object appears when tags detected
- [ ] "tracked targets" shows detected AprilTag positions
- [ ] Pose doesn't jump erratically

---

## 🐛 Common Issues & Solutions

### Issue: Cameras not appearing in code
**Check:**
- Camera names match exactly (case-sensitive)
- PhotonVision is running and connected to NetworkTables
- roboRIO has network connection to PhotonVision coprocessor

### Issue: Pose estimates are jumpy/inaccurate
**Solutions:**
- Increase standard deviation values
- Re-calibrate cameras
- Verify camera transforms are correct
- Check for motion blur (reduce exposure time)

### Issue: High latency warnings
**Solutions:**
- Reduce camera resolution
- Reduce frame rate
- Check CPU usage on PhotonVision device
- Verify network connection quality

### Issue: Tags not detected
**Check:**
- Lighting conditions (avoid backlighting)
- Camera exposure settings (manual exposure = 25)
- Camera is in focus
- AprilTag is within camera FOV
- Tag is not too far away (> 6m)

### Issue: 3D mode not working
**Check:**
- Both cameras have exact same resolution (640x480)
- Both cameras are calibrated at 640x480
- 3D mode is enabled in PhotonVision settings

---

## 📊 Data to Log for Analysis

When testing, log these values to CSV/WPILib DataLog:

1. **Timestamps**
   - Vision measurement timestamps
   - Odometry update timestamps

2. **Pose Data**
   - Odometry-only pose (X, Y, Theta)
   - Vision-corrected pose (X, Y, Theta)
   - Difference between the two

3. **Vision Quality Metrics**
   - Number of tags detected (per camera)
   - Average distance to tags
   - Pose ambiguity values
   - Standard deviations used

4. **Performance Metrics**
   - Vision update rate (Hz)
   - Latency (ms)
   - CPU usage (if available)

---

## ✅ Final Verification

- [ ] Both cameras provide stable pose estimates
- [ ] Odometry drift is minimal during teleop
- [ ] Autonomous paths execute accurately
- [ ] No persistent latency warnings
- [ ] Robot can localize from any starting position with AprilTags visible
- [ ] Code is committed to version control with meaningful commit message

---

## 📝 Notes Section

Use this space to record your measurements and observations:

### Actual Measurements:
```
Front Left Camera:
  Position: X=_____ Y=_____ Z=_____
  Rotation: Pitch=_____ Yaw=_____

Front Right Camera:
  Position: X=_____ Y=_____ Z=_____
  Rotation: Pitch=_____ Yaw=_____
```

### Testing Results:
```
Single Tag Detection Distance: _____m
Multi-Tag Detection Distance: _____m
Average Latency: _____ms
Pose Estimation Accuracy: +/- _____cm

Issues Found:
-
-

Solutions Applied:
-
-
```

### Tuned Standard Deviations:
```
SINGLE_TAG_STD_DEVS = VecBuilder.fill(___, ___, ___);
MULTI_TAG_STD_DEVS = VecBuilder.fill(___, ___, ___);
```
