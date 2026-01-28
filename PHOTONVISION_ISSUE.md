# PhotonVision Integration - Issue History & Resolution

**Date:** January 24-27, 2026
**Branch:** PhotonVisionTwoCameras
**Status:** ✅ RESOLVED

---

## Final Resolution (January 27, 2026)

### Issue: Dynamic Path to AprilTag Not Working with POV Right Button

**Problem:** When holding POV right on the controller, the robot would strafe instead of executing the dynamic path to the AprilTag, even when AprilTags were visible in Glass/PhotonVision.

**Root Cause:** The `seesAprilTag()` method and `SwervePathToAprilTagSupplier` were checking a **cached results list** that only contained unread results. Since the periodic loop already consumed all results via `getAllUnreadResults()`, the cache would be empty when POV right was pressed, causing the code to incorrectly believe no AprilTags were visible.

**Solution:** Modified both methods to call `camera.camera.getLatestResult()` directly from the PhotonCamera object instead of relying on the cached `resultsList`:

- `SwerveSubsystem.seesAprilTag()`: Now uses `camera.camera.getLatestResult()`
- `SwervePathToAprilTagSupplier.get()`: Now uses `camera.camera.getLatestResult()`

This ensures fresh data is retrieved from PhotonVision NetworkTables on-demand.

**Files Modified:**
- `src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java`
- `src/main/java/frc/robot/commands/swervedrive/SwervePathToAprilTagSupplier.java`

**Commit:** 3f93bc0 - "Remove Limelight, fix 2026 API deprecations, and fix PhotonVision TimeSync"

---

## Current System Configuration

### Version Information
- **PhotonVision (Raspberry Pi):** v2026.x
- **PhotonLib (RoboRIO):** v2026.1.1
- **WPILib:** 2026
- **Camera Pipeline:** AprilTag detection with MultiTag enabled

### Camera Configuration

**Front Left Camera ("FrontLeftCamera")**
- Position: 14.5" forward, 11.5" left, 9.25" up from robot center
- Rotation: Yaw -20.36° (angled inward to the right)
- Pitch: 0° (horizontal)

**Front Right Camera ("FrontRightCamera")**
- Position: 14.75" forward, 11.25" right, 9.25" up from robot center
- Rotation: Yaw +20.36° (angled inward to the left)
- Pitch: 0° (horizontal)

Both cameras are mounted on the front of the robot with convergent fields of view for optimal MultiTag detection.

### Pose Estimation Settings
- **Single Tag Std Devs:** [4, 4, 8] (x, y, theta)
- **Multi Tag Std Devs:** [0.5, 0.5, 1] (higher trust for multiple tags)

---

## Historical Investigation (January 24, 2026)

### Original Problem Statement

When starting the robot without looking at AprilTags, the odometry pose was not updated when AprilTags came into view, even though PhotonVision web UI showed tag detections.

### Initial Root Cause (v2025.3.1)

**PhotonVision v2025.3.1 with 3D mode enabled did NOT publish individual target data to NetworkTables.**

Evidence from logs:
```
[Vision Debug RAW] FrontLeftCamera | hasTargets=false | targetCount=0 | timestamp=2377.469 | multitagResult=EMPTY
```

- NetworkTables showed: `hasTarget=true`
- NetworkTables showed: `targetPose` was present
- NetworkTables MISSING: `targets` array (individual target data)
- PhotonLib received: `targetCount=0`

### Investigation Timeline

1. **First hypothesis:** Code logic issue with `setReferencePose()`
   - Removed deprecated `setReferencePose()` calls
   - Updated to use direct `poseEstimator.update()` method
   - Result: Still no targets detected

2. **Second hypothesis:** Version mismatch
   - Discovered PhotonVision v2025.3.1 vs PhotonLib v2026.0.1-beta mismatch
   - Downgraded PhotonLib to v2025.3.1
   - Result: Still no targets detected

3. **Third hypothesis:** PhotonVision configuration issue with v2025.3.1
   - Added extensive debug logging
   - Confirmed NetworkTables communication was working
   - Found `hasTarget=true` in NT but `targets` array missing
   - Result: PhotonVision v2025.3.1 doesn't publish target array when 3D mode is enabled

4. **Final solution:** Upgraded to PhotonVision v2026.x
   - Upgraded both PhotonVision (Pi) and PhotonLib (RoboRIO) to v2026
   - v2026 properly publishes target data with MultiTag support
   - Fixed cached result issue (see Final Resolution above)
   - Result: ✅ Vision system fully operational

---

## Code Changes Made

### Vision.java
- Removed deprecated `setReferencePose()` calls
- Updated to PhotonVision 2026 API using 2-argument `PhotonPoseEstimator` constructor
- Implemented first-tag detection with odometry reset
- Continuous vision updates with proper standard deviation handling
- TimeSync support via `getAllUnreadResults()` polling

### SwerveSubsystem.java
- Updated `seesAprilTag()` to use `camera.camera.getLatestResult()` directly
- Ensures fresh data from PhotonVision NetworkTables

### SwervePathToAprilTagSupplier.java
- Updated AprilTag ID retrieval to use `camera.camera.getLatestResult()` directly
- Fixes stale cache issue when POV buttons are pressed

### photonlib-v2026.1.1.json
- Upgraded from v2025.3.1 to v2026.1.1
- Matches PhotonVision v2026.x on Raspberry Pi

### Constants.java
- Updated camera positions to measured values
- Configured standard deviations for single-tag and multi-tag pose estimation

---

## Lessons Learned

1. **Version compatibility matters:** Keep PhotonVision and PhotonLib on the same major version
2. **v2025.3.1 3D mode limitation:** Does not publish individual target data to NetworkTables
3. **Cached vs. live results:** `getAllUnreadResults()` only returns NEW results; use `getLatestResult()` for on-demand queries
4. **Camera positioning:** Convergent fields of view improve MultiTag detection accuracy
5. **PhotonVision v2026 improvements:** Better target data publishing, MultiTag support, improved API

---

## References

- PhotonVision Docs: https://docs.photonvision.org/
- PhotonVision Releases: https://github.com/PhotonVision/photonvision/releases
- WPILib Coordinate System: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

## Related Files

- `src/main/java/frc/robot/subsystems/swervedrive/Vision.java` - Main vision code
- `src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java` - Swerve drive integration
- `src/main/java/frc/robot/commands/swervedrive/SwervePathToAprilTagSupplier.java` - Dynamic pathfinding to tags
- `vendordeps/photonlib-v2026.1.1.json` - PhotonLib version configuration
- `src/main/java/frc/robot/Constants.java` - Camera transforms and settings

---

**Created:** January 24, 2026
**Updated:** January 27, 2026
**Status:** Issue resolved, system operational on PhotonVision v2026
