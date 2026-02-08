package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for interfacing with the RealSense robot detector running on the coprocessor.
 * Subscribes to NetworkTables data published by the robot detector and provides
 * access to detected robot positions in field coordinates.
 */
public class RobotDetector extends SubsystemBase {
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable robotsTable;
    private final NetworkTable statusTable;

    // Robot detection subscribers
    private final IntegerSubscriber countSub;
    private final BooleanSubscriber validSub;
    private final DoubleArraySubscriber xSub;
    private final DoubleArraySubscriber ySub;
    private final DoubleArraySubscriber zSub;
    private final StringArraySubscriber allianceSub;

    // Status subscribers
    private final BooleanSubscriber healthySub;
    private final StringSubscriber cameraStateSub;

    /**
     * Creates a new RobotDetector subsystem.
     * Subscribes to all relevant NetworkTables topics.
     */
    public RobotDetector() {
        robotsTable = nt.getTable("RobotDetector/robots");
        statusTable = nt.getTable("RobotDetector/status");

        // Subscribe to robot detection data
        countSub = robotsTable.getIntegerTopic("count").subscribe(0);
        validSub = robotsTable.getBooleanTopic("valid").subscribe(false);
        xSub = robotsTable.getDoubleArrayTopic("x_m").subscribe(new double[]{});
        ySub = robotsTable.getDoubleArrayTopic("y_m").subscribe(new double[]{});
        zSub = robotsTable.getDoubleArrayTopic("z_m").subscribe(new double[]{});
        allianceSub = robotsTable.getStringArrayTopic("alliance").subscribe(new String[]{});

        // Subscribe to status data
        healthySub = statusTable.getBooleanTopic("healthy").subscribe(false);
        cameraStateSub = statusTable.getStringTopic("camera_state").subscribe("unknown");
    }

    /**
     * Checks if the robot detector camera is healthy and operating normally.
     * @return true if camera is healthy
     */
    public boolean isHealthy() {
        return healthySub.get();
    }

    /**
     * Checks if any robots are currently detected.
     * @return true if at least one robot is detected
     */
    public boolean hasRobots() {
        return validSub.get();
    }

    /**
     * Gets the number of robots currently detected.
     * @return number of robots (0-6)
     */
    public int getRobotCount() {
        return (int) countSub.get();
    }

    /**
     * Gets the camera state.
     * @return "running", "reconnecting", "failed", "stopped", or "unknown"
     */
    public String getCameraState() {
        return cameraStateSub.get();
    }

    /**
     * Gets the closest detected robot's position in camera frame coordinates.
     * Camera frame: X=right, Y=down, Z=forward (depth)
     * @return [x, y, z] in meters, or null if no robots detected
     */
    public double[] getClosestRobotCameraPosition() {
        if (!validSub.get()) return null;

        double[] x = xSub.get();
        double[] y = ySub.get();
        double[] z = zSub.get();

        if (x.length == 0) return null;

        return new double[]{x[0], y[0], z[0]};
    }

    /**
     * Gets the closest detected robot's distance from the camera.
     * @return distance in meters, or -1 if no robots detected
     */
    public double getClosestRobotDistance() {
        if (!validSub.get()) return -1;
        double[] z = zSub.get();
        return z.length > 0 ? z[0] : -1;
    }

    /**
     * Gets the closest detected robot's alliance color.
     * @return "red", "blue", or null if no robots detected
     */
    public String getClosestRobotAlliance() {
        if (!validSub.get()) return null;
        String[] alliances = allianceSub.get();
        return alliances.length > 0 ? alliances[0] : null;
    }

    /**
     * Gets the closest opponent robot's position in camera frame coordinates.
     * Filters for robots from the opposing alliance.
     * @return [x, y, z] in meters, or null if no opponent robots detected
     */
    public double[] getClosestOpponentRobotCameraPosition() {
        if (!validSub.get()) return null;

        var ourAlliance = DriverStation.getAlliance();
        if (ourAlliance.isEmpty()) return null;

        String opponentColor = (ourAlliance.get() == Alliance.Red) ? "blue" : "red";

        double[] x = xSub.get();
        double[] y = ySub.get();
        double[] z = zSub.get();
        String[] alliances = allianceSub.get();

        // Find first opponent robot (arrays are sorted by distance)
        for (int i = 0; i < alliances.length; i++) {
            if (alliances[i].equals(opponentColor)) {
                return new double[]{x[i], y[i], z[i]};
            }
        }

        return null;
    }

    /**
     * Gets all detected robots of a specific alliance.
     * @param alliance "red" or "blue"
     * @return Array of [x, y, z] positions in camera frame, sorted by distance
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

    /**
     * Converts a camera frame position to a field-relative pose.
     * This requires knowing the camera's pose on the robot and the robot's pose on the field.
     *
     * Camera frame: X=right, Y=down, Z=forward
     * Robot frame: X=forward, Y=left, Z=up
     *
     * @param cameraPos [x, y, z] position in camera frame (meters)
     * @param robotPose current robot pose on field
     * @param cameraTransform camera's transform relative to robot center
     * @return Pose2d of detected robot in field coordinates
     */
    public static Pose2d cameraPositionToFieldPose(
            double[] cameraPos,
            Pose2d robotPose,
            Translation2d cameraOffset,
            Rotation2d cameraYaw) {

        // Extract camera frame coordinates
        double cam_x = cameraPos[0];  // right
        // cam_y (down) not used for 2D positioning
        double cam_z = cameraPos[2];  // forward (depth)

        // Convert camera frame to robot frame
        // Camera frame (from NetworkTables): X=right+, Y=down+, Z=forward+
        // Robot frame (WPILib): X=forward+, Y=left+, Z=up+
        // Camera Z (depth/forward) -> Robot X (forward)
        // Camera X (right) -> Robot -Y (right), since Robot +Y is left
        double robot_x = cam_z;
        double robot_y = -cam_x;

        // Create translation in robot frame
        Translation2d robotFrameTranslation = new Translation2d(robot_x, robot_y);

        // Apply camera yaw offset (if camera is rotated)
        Translation2d rotatedTranslation = robotFrameTranslation.rotateBy(cameraYaw);

        // Add camera mounting offset
        Translation2d totalOffset = cameraOffset.plus(rotatedTranslation);

        // Transform to field coordinates using robot pose
        Translation2d fieldTranslation = robotPose.getTranslation().plus(
            totalOffset.rotateBy(robotPose.getRotation())
        );

        // Assume detected robot has same rotation as our robot (approximation)
        // In reality, you'd need to track robot orientation from detection
        return new Pose2d(fieldTranslation, robotPose.getRotation());
    }

    /**
     * Prints debug information about the robot detector state.
     * Useful for diagnosing connection and detection issues.
     */
    public void printDebugInfo() {
        System.out.println("=== RobotDetector Debug Info ===");
        System.out.printf("Healthy: %b%n", isHealthy());
        System.out.printf("Camera State: %s%n", getCameraState());
        System.out.printf("Has Robots: %b%n", hasRobots());
        System.out.printf("Robot Count: %d%n", getRobotCount());

        if (hasRobots()) {
            double[] pos = getClosestRobotCameraPosition();
            String alliance = getClosestRobotAlliance();
            System.out.printf("Closest robot: [%.2f, %.2f, %.2f] m, alliance: %s%n",
                pos[0], pos[1], pos[2], alliance);

            double[] oppPos = getClosestOpponentRobotCameraPosition();
            System.out.printf("Closest opponent: %s%n",
                oppPos == null ? "none" : String.format("[%.2f, %.2f, %.2f] m", oppPos[0], oppPos[1], oppPos[2]));
        }
        System.out.println("==============================");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Can be used for telemetry or logging
    }
}
