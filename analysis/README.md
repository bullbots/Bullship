# Robot Log Analysis Scripts

This directory contains Python scripts for analyzing robot telemetry data from WPILib DataLog (.wpilog) files.

## Setup

Install required Python packages:
```bash
python -m pip install wpilib
```

This installs WPILib Python (robotpy) v2026.2.1 which includes the DataLog reader.

## Scripts

### analyze_apriltag_alignment.py

Analyzes AprilTag detection data to diagnose issues with dynamic pathfinding to AprilTags.

**Usage:**
```bash
# Analyze a specific log file
python analysis/analyze_apriltag_alignment.py path/to/logfile.wpilog

# Analyze the most recent log in ~/Desktop/robot_logs/
python analysis/analyze_apriltag_alignment.py
```

**Output:**
- Detection rate (percentage of time AprilTags are visible)
- Which AprilTag IDs were detected
- Average distance to detected tags
- Continuous detection periods
- Diagnostic verdict

**Example:**
```
Reading log file: FRC_20260130_023456.wpilog
============================================================
Found AprilTag detected: ID 352
Found AprilTag ID: ID 353
Found AprilTag distance: ID 354

Total log entries: 378

Collecting AprilTag data...
Found 7402 AprilTag data samples

============================================================
APRILTAG ALIGNMENT ANALYSIS
============================================================
Total samples:        7402
Detected samples:     2341
Detection rate:       31.6%
Detected tag IDs:     [1, 3, 7]
Average distance:     2.45 meters
============================================================

VERDICT:
[OK] MODERATE DETECTION RATE (31.6%)
   AprilTags intermittently visible - alignment may be inconsistent

DETECTION PERIODS:
  Period 1: 15.23s for 5.42s
  Period 2: 28.91s for 2.15s
  Period 3: 45.67s for 8.93s
```

**What to Test:**
1. Deploy updated robot code with AprilTag logging
2. Enable the robot and position it facing an AprilTag
3. Press and hold POV right on the controller
4. Observe if the robot vibrates (indicates seesAprilTag() returned true)
5. Retrieve and analyze the log file

### analyze_straightness.py

Analyzes gyro yaw data to determine if the robot drives straight during testing.

**Usage:**
```bash
# Analyze a specific log file
python analysis/analyze_straightness.py path/to/logfile.wpilog

# Analyze the most recent log in ~/Desktop/robot_logs/
python analysis/analyze_straightness.py
```

**Output:**
- Initial and final gyro yaw values (degrees)
- Total drift amount and direction
- Verdict on straightness (< 2° = very straight, > 10° = mechanical issue)

**Example:**
```
Reading log file: FRC_20260130_023456.wpilog
============================================================
Found gyro yaw entry: /swerve/gyro/yaw (ID: 352)
Total log entries: 378

Collecting gyro yaw data...
Found 7402 gyro yaw samples
Time range: 11.39s to 160.75s
Duration: 149.36s

============================================================
STRAIGHTNESS ANALYSIS
============================================================
Initial yaw:       24.05°
Final yaw:         25.60°
Total drift:        1.55°
Min yaw:           24.04°
Max yaw:           26.02°
Max deviation:      1.98°
============================================================

VERDICT:
[OK] VERY STRAIGHT - Drift < 2 degrees
```

## Log File Locations

### On RoboRIO
Logs are stored on USB drive at: `/media/sda1/logs/*.wpilog`

Retrieve logs via SCP:
```bash
scp lvuser@10.18.91.2:/media/sda1/logs/FRC_*.wpilog ~/Desktop/robot_logs/
```

### On Development PC
Default location: `~/Desktop/robot_logs/`

## DataLog Entries

The robot code logs the following entries:

### Swerve Drive Data
- `/swerve/gyro/yaw` - Gyro yaw angle in degrees
- `/swerve/module0/position` - Front left module position (meters)
- `/swerve/module0/velocity` - Front left module velocity (m/s)
- `/swerve/module0/angle` - Front left module angle (degrees)
- `/swerve/module1/*` - Front right module data
- `/swerve/module2/*` - Back left module data
- `/swerve/module3/*` - Back right module data

### Odometry Data
- `/odometry/x` - Robot X position on field (meters)
- `/odometry/y` - Robot Y position on field (meters)
- `/odometry/rotation` - Robot heading (degrees)

### AprilTag Detection Data
- `/apriltag/detected` - Boolean, true when any camera sees an AprilTag
- `/apriltag/id` - Integer, the fiducial ID of the detected tag (-1 if none)
- `/apriltag/distance` - Double, distance to the tag in meters (-1 if none)
- `/apriltag/robotRelativeX` - Double, X coordinate of the tag relative to robot (meters)
- `/apriltag/robotRelativeY` - Double, Y coordinate of the tag relative to robot (meters)

### Test Marker Data
- `/test/povRightPressed` - Boolean, true when POV Right button is pressed
- `/test/targetRotation` - Double, calculated target rotation for alignment (degrees)
- `/test/currentRotation` - Double, current robot rotation when target is calculated (degrees)

### Vision Odometry Reset Data
- `/vision/resetPoseX` - Double, X coordinate of vision-estimated pose used for first odometry reset (meters)
- `/vision/resetPoseY` - Double, Y coordinate of vision-estimated pose used for first odometry reset (meters)
- `/vision/resetPoseRotation` - Double, rotation of vision-estimated pose used for first odometry reset (degrees)

Additional telemetry is available via NetworkTables entries (NT:/SmartDashboard/swerve/*)

## Viewing Logs Graphically

Use AdvantageScope for visual log analysis:
- Download: https://github.com/Mechanical-Advantage/AdvantageScope/releases
- Open .wpilog files directly
- Graph gyro and module data over time
