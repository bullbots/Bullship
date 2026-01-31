# AprilTag Dynamic Alignment System - Requirements

> **LOCKED REQUIREMENTS DOCUMENT**
> **DO NOT EDIT** - This file contains frozen requirements for the AprilTag alignment system.
> Claude Code and other automated tools should NOT modify this file.
> Changes require explicit human approval and manual editing only.

## System Overview
Dynamic pathfinding system that enables the robot to autonomously navigate to and align with AprilTags using PhotonVision camera detection and PathPlanner dynamic paths.

## Functional Requirements

### FR1: AprilTag Detection
- **FR1.1**: System shall detect AprilTags using two PhotonVision cameras (FrontLeftCamera, FrontRightCamera)
- **FR1.2**: System shall continuously update AprilTag detection status at periodic intervals (20ms)
- **FR1.3**: System shall support the 2025 Reefscape AndyMark field layout

### FR2: User Interaction
- **FR2.1**: System shall activate when driver presses POV Right on Xbox controller
- **FR2.2**: System shall provide haptic feedback (controller vibration) when AprilTag is detected
- **FR2.3**: System shall fall back to manual strafe control if no AprilTag is detected
- **FR2.4**: System shall terminate path execution when POV Right is released

### FR3: Dynamic Pathfinding
- **FR3.1**: System shall generate dynamic paths from any starting position/orientation to AprilTag alignment position
- **FR3.2**: System shall calculate target position 0.406 meters forward from AprilTag (in tag's reference frame)
- **FR3.3**: System shall allow lateral offset of ±0.167 meters (controlled by direction parameter)
- **FR3.4**: System shall orient robot to face directly toward the AprilTag (180° from tag's facing direction)
- **FR3.5**: System shall support slow mode operation (speed = 0.2 of max) and normal mode (speed = 0.35 of max)
- **FR3.6**: System shall use robot-relative AprilTag positioning from PhotonVision instead of field layout lookups
- **FR3.7**: System shall work with any AprilTag ID detected by PhotonVision cameras
- **FR3.8**: Path velocity constraints:
  - Linear velocity: 35% of max chassis velocity (normal) or 20% (slow mode)
  - Linear acceleration: 4.0 m/s²
  - Angular velocity: Max chassis angular velocity
  - Angular acceleration: 720°/s²

### FR4: Vision-Based Odometry
- **FR4.1**: System shall reset odometry to vision-estimated pose on first AprilTag detection
- **FR4.2**: System shall continuously blend vision measurements with wheel odometry using configured standard deviations:
  - Single tag: Higher uncertainty (configured in VisionConstants)
  - Multi-tag: Lower uncertainty (configured in VisionConstants)
- **FR4.3**: System shall use PhotonVision 2026 API with TimeSync support
- **FR4.4**: System shall flush stale PhotonVision results on robot code startup

### FR5: Coordinate Frame Transformations
- **FR5.1**: System shall transform offset position from AprilTag's reference frame to field coordinate frame
- **FR5.2**: System shall calculate final robot rotation as AprilTag rotation + 180°
- **FR5.3**: System shall use WPILib Pose2d.transformBy() for coordinate transformations

## Performance Requirements

### PR1: Alignment Accuracy
- **PR1.1**: Final robot heading shall be within ±5° of target rotation
- **PR1.2**: Final robot position shall be within ±0.02 meters of target position
- **PR1.3**: System shall complete alignment within 5 seconds from typical starting positions (1-3 meters from tag)

### PR2: Latency
- **PR2.1**: Vision pose estimates shall have average latency <50ms
- **PR2.2**: Path recalculation shall occur within 50ms of POV Right press

## Data Logging Requirements

### DL1: AprilTag Detection Logging
- **DL1.1**: Log AprilTag detection status (boolean) at 20ms intervals
- **DL1.2**: Log detected AprilTag ID (integer, -1 if none)
- **DL1.3**: Log distance to AprilTag (meters, -1 if none)
- **DL1.4**: Log AprilTag robot-relative position (X, Y in meters in robot's reference frame, -1 if none)

### DL2: Test Marker Logging
- **DL2.1**: Log POV Right button state for test period identification
- **DL2.2**: Log calculated target rotation (degrees)
- **DL2.3**: Log current robot rotation (degrees)

### DL3: Swerve Drive Logging
- **DL3.1**: Log gyro yaw angle (degrees)
- **DL3.2**: Log all four swerve module states (position, velocity, angle)
- **DL3.3**: Log robot odometry pose (X, Y in meters, rotation in degrees)

### DL4: Log File Management
- **DL4.1**: Logs shall be stored in .wpilog format on RoboRIO USB drive (/media/sda1/logs/)
- **DL4.2**: Log files shall include timestamp in filename (FRC_YYYYMMDD_HHMMSS.wpilog)

## Analysis Requirements

### AR1: Python Analysis Scripts
- **AR1.1**: Provide analyze_apriltag_alignment.py for detection rate and rotation analysis
- **AR1.2**: Provide analyze_straightness.py for gyro drift analysis
- **AR1.3**: Scripts shall auto-detect newest log file if none specified
- **AR1.4**: Scripts shall output human-readable diagnostics and verdicts

### AR2: Analysis Metrics
- **AR2.1**: Calculate overall detection rate (%)
- **AR2.2**: Calculate detection rate during specific test periods (POV Right pressed)
- **AR2.3**: Calculate average rotation error (target - current)
- **AR2.4**: Identify continuous detection periods
- **AR2.5**: Display sample rotation data points with timestamps

### AR3: Diagnostic Outputs
- **AR3.1**: Detection rate verdict: Good (>75%), Moderate (25-75%), Low (<25%), None (0%)
- **AR3.2**: Rotation error verdict: OK (<2°), Warning (2-5°), Error (>5°)
- **AR3.3**: Identify direction of systematic rotation error (CW/CCW)

## Hardware Configuration

### HW1: Camera Setup
- **HW1.1**: Two Arducam OV9281 cameras (640x480 @ 100 FPS)
- **HW1.2**: Camera diagonal FOV: ~90°
- **HW1.3**: Front left camera transform (relative to robot center):
  - Position: Configured in VisionConstants.FRONT_LEFT_CAMERA_POSITION
  - Rotation: Configured in VisionConstants.FRONT_LEFT_CAMERA_ROTATION
- **HW1.4**: Front right camera transform (relative to robot center):
  - Position: Configured in VisionConstants.FRONT_RIGHT_CAMERA_POSITION
  - Rotation: Configured in VisionConstants.FRONT_RIGHT_CAMERA_ROTATION

### HW2: PhotonVision
- **HW2.1**: PhotonVision version: v2026.1.1
- **HW2.2**: Running on Raspberry Pi coprocessor
- **HW2.3**: PhotonVision web interface accessible at photonvision.local:5800
- **HW2.4**: AprilTag pipeline configured and active

### HW3: Robot Platform
- **HW3.1**: Swerve drive with YAGSL (Yet Another Generic Swerve Library)
- **HW3.2**: Pigeon 2.0 IMU for gyro measurements
- **HW3.3**: Team: 1891

## Software Dependencies

### SD1: WPILib
- **SD1.1**: WPILib 2026 with updated geometry APIs
- **SD1.2**: AprilTagFieldLayout with k2025ReefscapeAndyMark support

### SD2: PhotonVision Library
- **SD2.1**: PhotonLib v2026.1.1
- **SD2.2**: PhotonPoseEstimator with 2-argument constructor (2026 API)
- **SD2.3**: Support for estimateCoprocMultiTagPose() and estimateLowestAmbiguityPose()

### SD3: PathPlanner
- **SD3.1**: PathPlanner AutoBuilder for dynamic pathfinding
- **SD3.2**: PathConstraints for velocity/acceleration limits

### SD4: Python Analysis
- **SD4.1**: Python 3.8+
- **SD4.2**: robotpy-wpilib (WPILib Python) v2026.2.1
- **SD4.3**: wpiutil.log.DataLogReader for .wpilog file parsing

## Non-Functional Requirements

### NFR1: Reliability
- **NFR1.1**: System shall gracefully handle AprilTag detection loss
- **NFR1.2**: System shall not crash if PhotonVision is unavailable
- **NFR1.3**: System shall flush stale camera data on startup

### NFR2: Maintainability
- **NFR2.1**: Code shall include comments explaining coordinate transformations
- **NFR2.2**: Logging shall provide sufficient data for post-match debugging
- **NFR2.3**: Analysis scripts shall provide clear diagnostic messages

### NFR3: Safety
- **NFR3.1**: System shall respect velocity and acceleration limits
- **NFR3.2**: Controller shall provide haptic feedback for system state awareness
- **NFR3.3**: Driver shall be able to override/cancel path execution by releasing POV Right

## Known Constraints

### C1: Camera Field of View
- **C1.1**: Two cameras have convergent fields of view (angled inward)
- **C1.2**: AprilTag detection limited to narrow forward cone
- **C1.3**: Detection rate may drop significantly outside optimal viewing angle

### C2: PhotonVision Latency
- **C2.1**: Average vision latency: 35ms (configured in camera properties)
- **C2.2**: Latency standard deviation: 5ms

## Success Criteria

### SC1: Functional Success
- System detects AprilTag 19 with >75% detection rate during test periods
- Robot completes dynamic path and aligns within ±5° rotation error (measured)
- Controller vibrates when AprilTag is detected
- System works from arbitrary starting positions and orientations

### SC2: Odometry Accuracy (Physical Verification)
- **SC2.1**: Final robot position measured with tape measure matches logged odometry within ±0.05 meters
- **SC2.2**: Final robot heading measured with protractor/visual alignment matches logged odometry within ±5°
- **SC2.3**: Vision-based pose estimates remain consistent (no sudden jumps >0.3m in logs during alignment)
- **SC2.4**: AprilTag-relative position logs match physical measurements when robot is stationary

**Verification Method:**
1. Place AprilTag 19 at known field position
2. Mark target alignment position on floor (0.406m from tag)
3. Run alignment sequence while logging
4. Measure final robot position and heading physically
5. Compare physical measurements to logged odometry values
6. Repeat from 3+ different starting positions

### SC3: Analysis Success
- Python scripts successfully parse all log files
- Rotation analysis identifies systematic errors
- Diagnostic verdicts accurately reflect system performance
- Odometry logs show smooth path execution without oscillation

### SC4: Operational Success
- Drivers can reliably trigger alignment with single button press
- System provides clear feedback (vibration) for detection status
- Path execution completes smoothly without jerking or oscillation
