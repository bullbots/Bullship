#!/usr/bin/env python3
"""
Analyze robot straightness from WPILib DataLog file.
Checks gyro yaw drift during straight-line driving tests.
"""

from wpiutil.log import DataLogReader
import sys

def analyze_log(log_path):
    """Analyze gyro yaw data to determine if robot drove straight."""

    print(f"Reading log file: {log_path}")
    print("=" * 60)

    reader = DataLogReader(log_path)

    gyro_entry_id = None
    entry_map = {}

    # First pass: build entry ID mapping from START records
    for record in reader:
        if record.isStart():
            entry = record.getStartData()
            # Use entry.entry (not record.getEntry() which returns 0 on START records)
            entry_id = entry.entry
            entry_map[entry_id] = (entry.name, entry.type)

            if entry.name == "/swerve/gyro/yaw":
                gyro_entry_id = entry_id
                print(f"Found gyro yaw entry: {entry.name} (ID: {gyro_entry_id})")

    if gyro_entry_id is None:
        print("ERROR: Could not find /swerve/gyro/yaw entry!")
        print(f"Available entries: {len(entry_map)}")
        # Print first 10 entries for debugging
        print("\nFirst 10 entries:")
        for i, (eid, (name, etype)) in enumerate(sorted(entry_map.items())[:10]):
            print(f"  {eid}: {name} ({etype})")
        return

    print(f"Total log entries: {len(entry_map)}")
    print("\nCollecting gyro yaw data...")

    # Second pass: collect gyro data from DATA records
    reader = DataLogReader(log_path)
    gyro_data = []

    for record in reader:
        # Skip non-data records
        if record.isStart() or record.isFinish() or record.isControl() or record.isSetMetadata():
            continue

        # This is a data record - use record.getEntry() which works correctly on data records
        entry_id = record.getEntry()

        if entry_id == gyro_entry_id:
            timestamp = record.getTimestamp() / 1_000_000.0  # Convert to seconds
            try:
                value = record.getDouble()
                gyro_data.append((timestamp, value))
            except Exception as e:
                print(f"Warning: Could not read gyro value at {timestamp:.3f}s: {e}")

    if not gyro_data:
        print("ERROR: No gyro data found!")
        print(f"Looked for entry ID: {gyro_entry_id}")
        return

    print(f"Found {len(gyro_data)} gyro yaw samples")
    print(f"Time range: {gyro_data[0][0]:.2f}s to {gyro_data[-1][0]:.2f}s")
    print(f"Duration: {gyro_data[-1][0] - gyro_data[0][0]:.2f}s")

    # Calculate statistics
    yaw_values = [yaw for _, yaw in gyro_data]
    initial_yaw = yaw_values[0]
    final_yaw = yaw_values[-1]
    min_yaw = min(yaw_values)
    max_yaw = max(yaw_values)

    total_drift = final_yaw - initial_yaw
    max_deviation = max_yaw - min_yaw

    print("\n" + "=" * 60)
    print("STRAIGHTNESS ANALYSIS")
    print("=" * 60)
    print(f"Initial yaw:     {initial_yaw:7.2f}°")
    print(f"Final yaw:       {final_yaw:7.2f}°")
    print(f"Total drift:     {total_drift:7.2f}°")
    print(f"Min yaw:         {min_yaw:7.2f}°")
    print(f"Max yaw:         {max_yaw:7.2f}°")
    print(f"Max deviation:   {max_deviation:7.2f}°")
    print("=" * 60)

    # Verdict
    print("\nVERDICT:")
    if abs(total_drift) < 2.0:
        print("[OK] VERY STRAIGHT - Drift < 2 degrees")
    elif abs(total_drift) < 5.0:
        print("[OK] REASONABLY STRAIGHT - Drift < 5 degrees")
    elif abs(total_drift) < 10.0:
        print("[WARN] MODERATE DRIFT - Check mechanical alignment (5-10 degrees)")
    else:
        print("[ERROR] SIGNIFICANT DRIFT - Mechanical issue likely (>10 degrees)")

    if abs(total_drift) > 2.0:
        direction = "RIGHT" if total_drift > 0 else "LEFT"
        print(f"   Robot drifted {direction} by {abs(total_drift):.2f} degrees")

    return gyro_data

if __name__ == "__main__":
    import os

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
                print("Usage: python analyze_straightness.py [path/to/logfile.wpilog]")
                sys.exit(1)
        else:
            print("ERROR: ~/Desktop/robot_logs/ directory not found")
            print("Usage: python analyze_straightness.py [path/to/logfile.wpilog]")
            sys.exit(1)

    analyze_log(log_file)
