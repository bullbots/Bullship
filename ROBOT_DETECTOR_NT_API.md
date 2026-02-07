# Robot Detector NetworkTables API

This document describes the NetworkTables interface for the RealSense robot detector running on the coprocessor. Use this to integrate robot detection into your robot code.

## Overview

The robot detector identifies FRC robots by their red/blue bumpers and publishes their 3D positions. It can track up to 6 robots simultaneously.

## NetworkTables Structure

All data is published under the `RobotDetector/` table.

### Robot Detections (`RobotDetector/robots/`)

| Topic | Type | Description |
|-------|------|-------------|
| `count` | int | Number of robots detected (0-6) |
| `valid` | boolean | True if any robots are detected |
| `x_m[]` | double[] | X positions in meters (camera frame, right is positive) |
| `y_m[]` | double[] | Y positions in meters (camera frame, down is positive) |
| `z_m[]` | double[] | Z positions (depth) in meters (forward is positive) |
| `pixel_x[]` | int[] | Pixel X coordinates of centroids |
| `pixel_y[]` | int[] | Pixel Y coordinates of centroids |
| `alliance[]` | string[] | Alliance color for each robot: `"red"` or `"blue"` |
| `confidence[]` | double[] | Confidence scores (0.0 - 1.0) for each detection |

**Note:** All arrays are sorted by distance (closest robot first). Arrays have the same length as `count`.

### Camera Status (`RobotDetector/status/`)

| Topic | Type | Description |
|-------|------|-------------|
| `healthy` | boolean | **Primary health check** - True if camera is working normally |
| `camera_connected` | boolean | True if camera is connected |
| `camera_state` | string | `"running"`, `"reconnecting"`, `"failed"`, or `"stopped"` |
| `time_since_frame_sec` | double | Seconds since last valid frame |
| `reconnect_attempts` | int | Number of reconnection attempts |
| `error_message` | string | Error description if any |

### Timing (`RobotDetector/timing/`)

| Topic | Type | Description |
|-------|------|-------------|
| `capture_timestamp_ms` | double | Camera hardware timestamp when frame was captured |
| `processing_time_ms` | double | Time to process the frame (ms) |
| `total_latency_ms` | double | Total latency from capture to publish (ms) |
| `publish_timestamp_ms` | double | System time when data was published (ms) |
| `fps` | double | Current processing framerate |

## Java Example (WPILib)

```java
import edu.wpi.first.networktables.*;

public class RobotDetectorSubscriber {
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable robotsTable;

    // Subscribers
    private final IntegerSubscriber countSub;
    private final BooleanSubscriber validSub;
    private final DoubleArraySubscriber xSub;
    private final DoubleArraySubscriber ySub;
    private final DoubleArraySubscriber zSub;
    private final StringArraySubscriber allianceSub;
    private final DoubleArraySubscriber confidenceSub;
    private final BooleanSubscriber healthySub;

    public RobotDetectorSubscriber() {
        robotsTable = nt.getTable("RobotDetector/robots");

        countSub = robotsTable.getIntegerTopic("count").subscribe(0);
        validSub = robotsTable.getBooleanTopic("valid").subscribe(false);
        xSub = robotsTable.getDoubleArrayTopic("x_m").subscribe(new double[]{});
        ySub = robotsTable.getDoubleArrayTopic("y_m").subscribe(new double[]{});
        zSub = robotsTable.getDoubleArrayTopic("z_m").subscribe(new double[]{});
        allianceSub = robotsTable.getStringArrayTopic("alliance").subscribe(new String[]{});
        confidenceSub = robotsTable.getDoubleArrayTopic("confidence").subscribe(new double[]{});

        healthySub = nt.getTable("RobotDetector/status")
            .getBooleanTopic("healthy").subscribe(false);
    }

    public boolean isHealthy() {
        return healthySub.get();
    }

    public boolean hasRobots() {
        return validSub.get();
    }

    public int getRobotCount() {
        return (int) countSub.get();
    }

    /**
     * Get the closest robot's position.
     * @return [x, y, z] in meters, or null if no robots detected
     */
    public double[] getClosestRobotPosition() {
        if (!validSub.get()) return null;

        double[] x = xSub.get();
        double[] y = ySub.get();
        double[] z = zSub.get();

        if (x.length == 0) return null;

        return new double[]{x[0], y[0], z[0]};
    }

    /**
     * Get the closest robot's distance in meters.
     */
    public double getClosestRobotDistance() {
        if (!validSub.get()) return -1;
        double[] z = zSub.get();
        return z.length > 0 ? z[0] : -1;
    }

    /**
     * Get the closest robot's alliance color.
     * @return "red", "blue", or null
     */
    public String getClosestRobotAlliance() {
        if (!validSub.get()) return null;
        String[] alliances = allianceSub.get();
        return alliances.length > 0 ? alliances[0] : null;
    }

    /**
     * Get all detected robots of a specific alliance.
     * @param alliance "red" or "blue"
     * @return Array of [x, y, z] positions
     */
    public double[][] getRobotsByAlliance(String alliance) {
        double[] x = xSub.get();
        double[] y = ySub.get();
        double[] z = zSub.get();
        String[] alliances = allianceSub.get();

        int count = 0;
        for (String a : alliances) {
            if (a.equals(alliance)) count++;
        }

        double[][] result = new double[count][3];
        int idx = 0;
        for (int i = 0; i < alliances.length; i++) {
            if (alliances[i].equals(alliance)) {
                result[idx][0] = x[i];
                result[idx][1] = y[i];
                result[idx][2] = z[i];
                idx++;
            }
        }
        return result;
    }
}
```

## Coordinate System

The camera uses a right-handed coordinate system:
- **X**: Right is positive (meters)
- **Y**: Down is positive (meters)
- **Z**: Forward/depth is positive (meters)

To convert to field coordinates, you'll need to apply the camera's pose transform based on where it's mounted on your robot.

## Tuning Parameters

These can be adjusted via NetworkTables at `RobotDetector/tuning/`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_depth_m` | 0.3 | Minimum detection distance (m) |
| `max_depth_m` | 5.0 | Maximum detection distance (m) |
| `min_area` | 2000 | Minimum contour area (pixels) |
| `min_aspect_ratio` | 1.5 | Minimum bumper aspect ratio |
| `max_aspect_ratio` | 6.0 | Maximum bumper aspect ratio |
| `min_solidity` | 0.6 | Minimum contour solidity (0-1) |
| `smoothing` | 0.3 | Temporal smoothing (0=none, 0.9=heavy) |
| `red_hue_low1/high1` | 0/10 | Red hue range 1 (wraps around) |
| `red_hue_low2/high2` | 170/179 | Red hue range 2 |
| `blue_hue_low/high` | 100/130 | Blue hue range |

## Debug Streams

MJPEG streams available on the coprocessor (replace `<IP>` with coprocessor IP):

| Stream | URL | Description |
|--------|-----|-------------|
| Color | `http://<IP>:1181` | Raw camera image |
| Depth | `http://<IP>:1182` | Depth colormap |
| Color Mask | `http://<IP>:1183` | Red/blue color detection |
| Depth Mask | `http://<IP>:1184` | Depth range filter |
| Combined | `http://<IP>:1185` | Color + depth combined |
| Result | `http://<IP>:1186` | Final detections with overlays |

Stream toggles can be controlled via `RobotDetector/debug/stream_*` boolean topics.

## Connection

The robot detector connects as an NT4 client to port 5810 by default. Your robot code should be running an NT4 server (standard WPILib configuration).

```bash
# On coprocessor:
./robot_detector.py <ROBORIO_IP>
```
