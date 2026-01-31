#!/usr/bin/env python3
"""
Analyze AprilTag alignment from WPILib DataLog file.
Checks if AprilTags are detected when POV right is pressed and analyzes alignment quality.
"""

from wpiutil.log import DataLogReader
import sys
import os


def analyze_log(log_path):
    """Analyze AprilTag detection and alignment data."""

    print(f"Reading log file: {log_path}")
    print("=" * 60)

    reader = DataLogReader(log_path)

    # Entry IDs we're looking for
    entry_map = {}
    apriltag_detected_id = None
    apriltag_id_id = None
    apriltag_distance_id = None
    target_pose_x_id = None
    target_pose_y_id = None
    robot_pose_x_id = None
    robot_pose_y_id = None
    gyro_yaw_id = None
    pov_right_pressed_id = None
    target_rotation_id = None
    current_rotation_id = None
    vision_reset_pose_x_id = None
    vision_reset_pose_y_id = None
    vision_reset_pose_rotation_id = None

    # First pass: find all entry IDs
    for record in reader:
        if record.isStart():
            entry = record.getStartData()
            entry_id = entry.entry
            entry_map[entry_id] = (entry.name, entry.type)

            if entry.name == "/apriltag/detected":
                apriltag_detected_id = entry_id
                print(f"Found AprilTag detected: ID {entry_id}")
            elif entry.name == "/apriltag/id":
                apriltag_id_id = entry_id
                print(f"Found AprilTag ID: ID {entry_id}")
            elif entry.name == "/apriltag/distance":
                apriltag_distance_id = entry_id
                print(f"Found AprilTag distance: ID {entry_id}")
            elif entry.name == "/apriltag/targetPoseX":
                target_pose_x_id = entry_id
                print(f"Found target pose X: ID {entry_id}")
            elif entry.name == "/apriltag/targetPoseY":
                target_pose_y_id = entry_id
                print(f"Found target pose Y: ID {entry_id}")
            elif entry.name == "/swerve/gyro/yaw":
                gyro_yaw_id = entry_id
                print(f"Found gyro yaw: ID {entry_id}")
            elif entry.name == "/test/povRightPressed":
                pov_right_pressed_id = entry_id
                print(f"Found POV right pressed marker: ID {entry_id}")
            elif entry.name == "/test/targetRotation":
                target_rotation_id = entry_id
                print(f"Found target rotation: ID {entry_id}")
            elif entry.name == "/test/currentRotation":
                current_rotation_id = entry_id
                print(f"Found current rotation: ID {entry_id}")
            elif entry.name == "/vision/resetPoseX":
                vision_reset_pose_x_id = entry_id
                print(f"Found vision reset pose X: ID {entry_id}")
            elif entry.name == "/vision/resetPoseY":
                vision_reset_pose_y_id = entry_id
                print(f"Found vision reset pose Y: ID {entry_id}")
            elif entry.name == "/vision/resetPoseRotation":
                vision_reset_pose_rotation_id = entry_id
                print(f"Found vision reset pose rotation: ID {entry_id}")

    print(f"\nTotal log entries: {len(entry_map)}")

    # Check if we found the AprilTag entries
    if apriltag_detected_id is None:
        print("\nERROR: No AprilTag logging entries found!")
        print("Make sure the robot code with AprilTag logging has been deployed and run.")
        return

    print("\nCollecting AprilTag data...")

    # Second pass: collect data
    reader = DataLogReader(log_path)
    apriltag_data = []
    rotation_data = []

    for record in reader:
        if record.isStart() or record.isFinish() or record.isControl() or record.isSetMetadata():
            continue

        entry_id = record.getEntry()
        timestamp = record.getTimestamp() / 1_000_000.0

        # Collect AprilTag detection data
        if entry_id == apriltag_detected_id:
            detected = record.getBoolean()
            apriltag_data.append({
                'timestamp': timestamp,
                'detected': detected,
                'id': -1,
                'distance': -1.0,
                'target_x': -1.0,
                'target_y': -1.0
            })
        # Collect rotation data
        elif entry_id == target_rotation_id:
            target_rot = record.getDouble()
            rotation_data.append({
                'timestamp': timestamp,
                'target_rotation': target_rot,
                'current_rotation': None
            })

    # Third pass: fill in other fields
    reader = DataLogReader(log_path)
    data_index = 0
    rot_index = 0

    for record in reader:
        if record.isStart() or record.isFinish() or record.isControl() or record.isSetMetadata():
            continue

        entry_id = record.getEntry()
        timestamp = record.getTimestamp() / 1_000_000.0

        # Find matching timestamp in apriltag_data
        while data_index < len(apriltag_data) and apriltag_data[data_index]['timestamp'] < timestamp - 0.001:
            data_index += 1

        if data_index < len(apriltag_data):
            current_data = apriltag_data[data_index]

            if abs(current_data['timestamp'] - timestamp) < 0.001:
                if entry_id == apriltag_id_id:
                    current_data['id'] = record.getInteger()
                elif entry_id == apriltag_distance_id:
                    current_data['distance'] = record.getDouble()
                elif entry_id == target_pose_x_id:
                    current_data['target_x'] = record.getDouble()
                elif entry_id == target_pose_y_id:
                    current_data['target_y'] = record.getDouble()

        # Fill in rotation data
        while rot_index < len(rotation_data) and rotation_data[rot_index]['timestamp'] < timestamp - 0.001:
            rot_index += 1

        if rot_index < len(rotation_data):
            current_rot_data = rotation_data[rot_index]

            if abs(current_rot_data['timestamp'] - timestamp) < 0.001:
                if entry_id == current_rotation_id:
                    current_rot_data['current_rotation'] = record.getDouble()

    if not apriltag_data:
        print("ERROR: No AprilTag data found!")
        return

    # Analyze the data
    print(f"\nFound {len(apriltag_data)} AprilTag data samples")
    print(f"Time range: {apriltag_data[0]['timestamp']:.2f}s to {apriltag_data[-1]['timestamp']:.2f}s")
    print(f"Duration: {apriltag_data[-1]['timestamp'] - apriltag_data[0]['timestamp']:.2f}s")

    # Count detections
    total_samples = len(apriltag_data)
    detected_samples = sum(1 for d in apriltag_data if d['detected'])
    detection_rate = (detected_samples / total_samples * 100) if total_samples > 0 else 0

    # Find detected tag IDs
    detected_ids = set(d['id'] for d in apriltag_data if d['detected'] and d['id'] >= 0)

    # Calculate average distance when detected
    distances = [d['distance'] for d in apriltag_data if d['detected'] and d['distance'] > 0]
    avg_distance = sum(distances) / len(distances) if distances else 0

    print("\n" + "=" * 60)
    print("APRILTAG ALIGNMENT ANALYSIS")
    print("=" * 60)
    print(f"Total samples:        {total_samples}")
    print(f"Detected samples:     {detected_samples}")
    print(f"Detection rate:       {detection_rate:.1f}%")
    print(f"Detected tag IDs:     {sorted(detected_ids) if detected_ids else 'None'}")
    print(f"Average distance:     {avg_distance:.2f} meters")
    print("=" * 60)

    # Verdict
    print("\nVERDICT:")
    if detection_rate == 0:
        print("[ERROR] NO APRILTAGS DETECTED")
        print("   Possible issues:")
        print("   - AprilTags not in camera field of view")
        print("   - Camera not connected or working")
        print("   - Lighting conditions too poor")
        print("   - PhotonVision not running")
    elif detection_rate < 25:
        print(f"[WARN] LOW DETECTION RATE ({detection_rate:.1f}%)")
        print("   AprilTags are barely visible - check camera positioning")
    elif detection_rate < 75:
        print(f"[OK] MODERATE DETECTION RATE ({detection_rate:.1f}%)")
        print("   AprilTags intermittently visible - alignment may be inconsistent")
    else:
        print(f"[OK] GOOD DETECTION RATE ({detection_rate:.1f}%)")
        print("   AprilTags consistently visible")

    # Find periods of continuous detection
    print("\nDETECTION PERIODS:")
    in_detection = False
    start_time = 0
    detection_periods = []

    for i, data in enumerate(apriltag_data):
        if data['detected'] and not in_detection:
            in_detection = True
            start_time = data['timestamp']
        elif not data['detected'] and in_detection:
            in_detection = False
            duration = apriltag_data[i-1]['timestamp'] - start_time
            detection_periods.append((start_time, duration))

    if in_detection:
        duration = apriltag_data[-1]['timestamp'] - start_time
        detection_periods.append((start_time, duration))

    if detection_periods:
        for i, (start, duration) in enumerate(detection_periods[:10]):  # Show first 10
            print(f"  Period {i+1}: {start:.2f}s for {duration:.2f}s")
        if len(detection_periods) > 10:
            print(f"  ... and {len(detection_periods) - 10} more periods")
    else:
        print("  No continuous detection periods found")

    # Show POV right pressed periods if available
    if pov_right_pressed_id is not None:
        print("\nPOV RIGHT TEST PERIODS:")
        pov_periods = find_pov_periods(log_path, pov_right_pressed_id)
        if pov_periods:
            for i, (start, duration) in enumerate(pov_periods):
                print(f"  Test {i+1}: {start:.2f}s for {duration:.2f}s")
                # Find detection rate during this period
                test_samples = [d for d in apriltag_data if start <= d['timestamp'] <= start + duration]
                if test_samples:
                    test_detections = sum(1 for d in test_samples if d['detected'])
                    test_rate = (test_detections / len(test_samples) * 100) if test_samples else 0
                    print(f"           Detection rate during test: {test_rate:.1f}%")
        else:
            print("  No POV right button presses detected in log")

    # Analyze rotation data if available
    if rotation_data:
        print("\n" + "=" * 60)
        print("ROTATION ALIGNMENT ANALYSIS")
        print("=" * 60)

        # Filter to complete rotation data points
        complete_rotations = [r for r in rotation_data if r['current_rotation'] is not None]

        if complete_rotations:
            print(f"Rotation data points: {len(complete_rotations)}")

            # Calculate rotation differences
            rotation_diffs = []
            for r in complete_rotations:
                diff = r['target_rotation'] - r['current_rotation']
                # Normalize to -180 to 180
                while diff > 180:
                    diff -= 360
                while diff < -180:
                    diff += 360
                rotation_diffs.append(diff)

            avg_diff = sum(rotation_diffs) / len(rotation_diffs) if rotation_diffs else 0
            min_diff = min(rotation_diffs) if rotation_diffs else 0
            max_diff = max(rotation_diffs) if rotation_diffs else 0

            print(f"Average rotation difference: {avg_diff:.2f}° (target - current)")
            print(f"Min difference: {min_diff:.2f}°")
            print(f"Max difference: {max_diff:.2f}°")
            print(f"\nSample rotation targets:")
            for i, r in enumerate(complete_rotations[:5]):  # Show first 5
                diff = r['target_rotation'] - r['current_rotation']
                while diff > 180:
                    diff -= 360
                while diff < -180:
                    diff += 360
                print(f"  {i+1}. Target: {r['target_rotation']:6.2f}°, Current: {r['current_rotation']:6.2f}°, Diff: {diff:6.2f}°")

            if len(complete_rotations) > 5:
                print(f"  ... and {len(complete_rotations) - 5} more data points")

            print("\nINTERPRETATION:")
            if abs(avg_diff) < 2:
                print("  [OK] Target rotation calculations appear correct")
            elif abs(avg_diff) >= 2 and abs(avg_diff) < 5:
                print(f"  [WARN] Small systematic error in rotation: {avg_diff:.2f}°")
            else:
                print(f"  [ERROR] Large systematic rotation error: {avg_diff:.2f}°")
                if avg_diff > 0:
                    print("         Target rotation is consistently HIGHER than current")
                    print("         Robot will rotate CCW to reach target")
                else:
                    print("         Target rotation is consistently LOWER than current")
                    print("         Robot will rotate CW to reach target")
        else:
            print("  No complete rotation data points found")
        print("=" * 60)

    # Analyze vision reset pose data if available
    if vision_reset_pose_x_id is not None:
        print("\n" + "=" * 60)
        print("VISION ODOMETRY RESET ANALYSIS")
        print("=" * 60)

        # Collect vision reset pose data
        reader = DataLogReader(log_path)
        vision_reset_data = []

        for record in reader:
            if record.isStart() or record.isFinish() or record.isControl() or record.isSetMetadata():
                continue

            entry_id = record.getEntry()
            timestamp = record.getTimestamp() / 1_000_000.0

            if entry_id == vision_reset_pose_x_id:
                vision_reset_data.append({
                    'timestamp': timestamp,
                    'x': record.getDouble(),
                    'y': None,
                    'rotation': None
                })

        # Fill in Y and rotation values
        reader = DataLogReader(log_path)
        data_index = 0

        for record in reader:
            if record.isStart() or record.isFinish() or record.isControl() or record.isSetMetadata():
                continue

            entry_id = record.getEntry()
            timestamp = record.getTimestamp() / 1_000_000.0

            # Find matching timestamp in vision_reset_data
            while data_index < len(vision_reset_data) and vision_reset_data[data_index]['timestamp'] < timestamp - 0.001:
                data_index += 1

            if data_index < len(vision_reset_data):
                current_data = vision_reset_data[data_index]

                if abs(current_data['timestamp'] - timestamp) < 0.001:
                    if entry_id == vision_reset_pose_y_id:
                        current_data['y'] = record.getDouble()
                    elif entry_id == vision_reset_pose_rotation_id:
                        current_data['rotation'] = record.getDouble()

        # Display vision reset pose data
        complete_resets = [d for d in vision_reset_data if d['y'] is not None and d['rotation'] is not None]

        if complete_resets:
            print(f"Vision odometry resets: {len(complete_resets)}")
            print("\nRESET POSES FROM PHOTONVISION:")
            for i, reset in enumerate(complete_resets):
                print(f"  Reset {i+1} at {reset['timestamp']:.2f}s:")
                print(f"    X: {reset['x']:.3f} m")
                print(f"    Y: {reset['y']:.3f} m")
                print(f"    Rotation: {reset['rotation']:.2f}°")

            if len(complete_resets) > 1:
                print("\n[INFO] Multiple vision resets detected!")
                print("       This may indicate hasResetOdometryFromVision flag is not working correctly.")
        else:
            print("  No vision odometry resets found in this log")
            print("  This means the robot had already seen an AprilTag before this test,")
            print("  or PhotonVision is not providing pose estimates.")

        print("=" * 60)

    return apriltag_data


def find_pov_periods(log_path, pov_right_id):
    """Find periods when POV right was pressed."""
    reader = DataLogReader(log_path)
    pov_data = []

    for record in reader:
        if record.isStart() or record.isFinish() or record.isControl() or record.isSetMetadata():
            continue

        if record.getEntry() == pov_right_id:
            timestamp = record.getTimestamp() / 1_000_000.0
            pressed = record.getBoolean()
            pov_data.append((timestamp, pressed))

    # Find periods
    periods = []
    in_press = False
    start_time = 0

    for timestamp, pressed in pov_data:
        if pressed and not in_press:
            in_press = True
            start_time = timestamp
        elif not pressed and in_press:
            in_press = False
            duration = timestamp - start_time
            periods.append((start_time, duration))

    if in_press and pov_data:
        duration = pov_data[-1][0] - start_time
        periods.append((start_time, duration))

    return periods


if __name__ == "__main__":
    # Check if a log file was provided as command line argument
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        # Default to the most recent log in Desktop/robot_logs
        log_dir = os.path.join(os.path.expanduser("~"), "Desktop", "robot_logs")

        if os.path.exists(log_dir):
            # Find all .wpilog files
            log_files = [f for f in os.listdir(log_dir) if f.endswith('.wpilog')]

            if log_files:
                # Sort by modification time, newest first
                log_files.sort(key=lambda f: os.path.getmtime(os.path.join(log_dir, f)), reverse=True)
                log_file = os.path.join(log_dir, log_files[0])
                print(f"No log file specified, using most recent: {log_files[0]}\n")
            else:
                print("ERROR: No .wpilog files found in ~/Desktop/robot_logs/")
                print("Usage: python analyze_apriltag_alignment.py [path/to/logfile.wpilog]")
                sys.exit(1)
        else:
            print("ERROR: ~/Desktop/robot_logs/ directory not found")
            print("Usage: python analyze_apriltag_alignment.py [path/to/logfile.wpilog]")
            sys.exit(1)

    analyze_log(log_file)
