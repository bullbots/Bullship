# Testing AprilTag Alignment (POV Right Button)

This guide explains how to test and diagnose the AprilTag dynamic pathfinding feature when pressing POV right on the Xbox controller.

## Problem Statement

When holding POV right on the controller, the robot should:
1. Detect if an AprilTag is visible (`seesAprilTag()` returns true)
2. Vibrate the controller to confirm detection
3. Execute a dynamic path to align with the AprilTag

Currently, the robot strafes instead of executing the dynamic path, suggesting the AprilTag detection is failing.

## Setup

### 1. Deploy Updated Code

The code now includes AprilTag logging. Deploy it to the robot:

```bash
./gradlew deploy
```

### 2. Position Robot for Testing

- Place the robot facing an AprilTag (tags 1-8 for 2025 Reefscape field)
- Ensure the tag is within camera field of view
- Distance: 1-3 meters is ideal for testing

## Testing Procedure

### Step 1: Run the Test

1. Enable the robot in teleop mode
2. **Press and hold POV right** on the Xbox controller
3. **Observe the controller** - does it vibrate?
   - **Vibrates**: AprilTag detected, dynamic path should execute
   - **No vibration**: AprilTag NOT detected, will strafe instead (current issue)
4. Let the test run for 5-10 seconds
5. Disable the robot

### Step 2: Retrieve Log Files

From your development PC, copy the log files from the robot:

```bash
# List logs on robot
ssh lvuser@10.18.91.2 "ls -lht /media/sda1/logs/*.wpilog | head -5"

# Copy the newest log
scp lvuser@10.18.91.2:/media/sda1/logs/FRC_*.wpilog ~/Desktop/robot_logs/
```

### Step 3: Analyze the Logs

Run the AprilTag alignment analysis:

```bash
python analysis/analyze_apriltag_alignment.py
```

Or analyze a specific log file:

```bash
python analysis/analyze_apriltag_alignment.py ~/Desktop/robot_logs/FRC_20260130_023456.wpilog
```

## Interpreting Results

### Good Results (Working)

```
Detection rate:       85.3%
Detected tag IDs:     [1]
Average distance:     2.15 meters

VERDICT:
[OK] GOOD DETECTION RATE (85.3%)
   AprilTags consistently visible
```

**What this means:**
- AprilTag is visible most of the time
- `seesAprilTag()` should return true
- Controller should vibrate
- Dynamic path should execute

### Bad Results (Current Issue)

```
Detection rate:       0.0%
Detected tag IDs:     None
Average distance:     0.00 meters

VERDICT:
[ERROR] NO APRILTAGS DETECTED
   Possible issues:
   - AprilTags not in camera field of view
   - Camera not connected or working
   - Lighting conditions too poor
   - PhotonVision not running
```

**What this means:**
- No AprilTags are being detected
- `seesAprilTag()` returns false
- Controller will NOT vibrate
- Robot will strafe instead (fallback behavior)

## Common Issues and Fixes

### Issue 1: No AprilTags Detected

**Symptoms:**
- Detection rate = 0%
- Controller doesn't vibrate
- Robot strafes instead of dynamic path

**Possible Causes:**

1. **PhotonVision not running**
   - Check: Open PhotonVision web interface at `http://photonvision.local:5800`
   - Fix: Restart PhotonVision on the Raspberry Pi

2. **Camera disconnected**
   - Check: Look for cameras in PhotonVision interface
   - Fix: Check USB connections to Raspberry Pi

3. **Camera not aimed at tags**
   - Check: Use PhotonVision camera stream to see what cameras see
   - Fix: Adjust camera mounting positions/angles

4. **Wrong pipeline selected**
   - Check: Ensure AprilTag pipeline is active in PhotonVision
   - Fix: Select the AprilTag detection pipeline

5. **Lighting too poor/bright**
   - Check: Camera exposure settings in PhotonVision
   - Fix: Adjust exposure or lighting conditions

### Issue 2: Low/Intermittent Detection

**Symptoms:**
- Detection rate = 10-50%
- Controller vibrates sometimes
- Inconsistent behavior

**Possible Causes:**

1. **Camera field of view too narrow**
   - The two cameras have convergent fields of view (angled inward)
   - Tags may only be visible in a narrow forward cone
   - Fix: Adjust camera yaw angles to widen coverage

2. **Robot motion during detection**
   - Moving robot may lose sight of tags
   - Fix: Test while stationary first

3. **Tag at edge of field of view**
   - Tag may be intermittently visible
   - Fix: Position robot more directly facing the tag

## Code Reference

### POV Right Button Binding

Location: [RobotContainer.java:208-214](../src/main/java/frc/robot/RobotContainer.java#L208-L214)

```java
driverXbox.povRight().whileTrue(
    new ConditionalCommand(
        new ParallelDeadlineGroup(
            new DeferredCommand(new SwervePathToAprilTagSupplier(1.0, true, true), Set.of(drivebase)),
            new ControllerVibrate(50)),
        new StrafeAndMoveForward(drivebase, driveStrafeRight),
        drivebase::seesAprilTag));  // This determines which command runs!
```

**Key Point:** The `drivebase::seesAprilTag` condition determines:
- **True** → Execute dynamic path + vibrate controller
- **False** → Execute strafe command (fallback)

### AprilTag Detection Logic

Location: [SwerveSubsystem.java](../src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java)

```java
public boolean seesAprilTag() {
  // Check all PhotonVision cameras for AprilTag detections
  // Use camera.getLatestResult() directly from PhotonCamera instead of cached results
  for (Cameras camera : Cameras.values()) {
    PhotonPipelineResult result = camera.camera.getLatestResult();
    if (result.hasTargets()) {
      return true;
    }
  }
  return false;
}
```

This method:
1. Checks both cameras (FrontLeftCamera, FrontRightCamera)
2. Uses `camera.camera.getLatestResult()` to get fresh data from NetworkTables
3. Returns true if ANY camera sees ANY AprilTag

## Next Steps After Analysis

Once you've identified the issue:

1. **If detection rate is 0%:**
   - Check PhotonVision is running
   - Verify cameras are connected
   - Check camera streams show AprilTags

2. **If detection rate is low (<50%):**
   - Adjust camera angles for wider coverage
   - Verify correct pipeline is selected
   - Check lighting conditions

3. **If detection rate is good (>75%):**
   - The AprilTag detection is working!
   - If the dynamic path still doesn't execute, the issue is elsewhere
   - Check that controller vibration works (confirms `seesAprilTag()` is true)

## Additional Analysis

You can also check the straightness of the robot's movement:

```bash
python analysis/analyze_straightness.py
```

This helps verify there are no mechanical issues affecting path execution.
