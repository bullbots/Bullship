// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotDetector;

/**
 * Command supplier that generates a path to track a detected opponent robot.
 * Uses the RealSense robot detector running on the coprocessor to identify
 * opponent robots and generates a dynamic path to maintain a specific distance.
 *
 * Similar to SwervePathToAprilTagSupplier but:
 * - Uses X offset only (no Y offset)
 * - Targets opponent alliance robots
 * - Maintains 6 feet (~1.83m) distance
 */
public class SwervePathToDetectedRobotSupplier implements Supplier<Command> {

    private boolean isSlow;
    private double speed = 0.35;
    private static final double X_OFFSET_METERS = 1.8288; // 6 feet in meters

    /**
     * Creates a new robot tracking command supplier.
     * @param isSlow if true, uses slower speed (0.2 instead of 0.35)
     */
    public SwervePathToDetectedRobotSupplier(boolean isSlow) {
        this.isSlow = isSlow;
    }

    @Override
    public Command get() {
        System.out.println("=== SwervePathToDetectedRobotSupplier.get() CALLED ===");

        var drivebase = RobotContainer.drivebase;
        var robotDetector = RobotContainer.robotDetector;

        // Check if robot detector is healthy and detecting opponent robots
        boolean healthy = robotDetector.isHealthy();
        System.out.printf("Robot detector healthy: %b%n", healthy);
        if (!healthy) {
            System.out.println("ERROR: Robot detector not healthy!");
            return new PrintCommand("Robot detector not healthy!");
        }

        boolean hasRobots = robotDetector.hasRobots();
        int robotCount = robotDetector.getRobotCount();
        System.out.printf("Has robots: %b, Count: %d%n", hasRobots, robotCount);
        if (!hasRobots) {
            System.out.println("ERROR: No robots detected!");
            return new PrintCommand("No robots detected!");
        }

        // Get closest opponent robot position in camera frame
        double[] opponentPos = robotDetector.getClosestOpponentRobotCameraPosition();
        System.out.printf("Opponent position (camera frame): %s%n",
            opponentPos == null ? "null" : String.format("X=%.2f (right+), Y=%.2f (down+), Z=%.2f (forward+)",
                opponentPos[0], opponentPos[1], opponentPos[2]));

        if (opponentPos == null) {
            System.out.println("ERROR: No opponent robots detected!");
            return new PrintCommand("No opponent robots detected!");
        }

        // Get current robot pose
        Pose2d currentPose = drivebase.getPose();

        // Convert camera position to field coordinates
        // NOTE: You'll need to provide the correct camera offset and yaw for your setup
        // These values should match your camera mounting position
        Translation2d cameraOffset = new Translation2d(
            Units.inchesToMeters(14.5),  // Forward from robot center (adjust as needed)
            Units.inchesToMeters(0)      // Lateral offset (adjust as needed)
        );
        Rotation2d cameraYaw = Rotation2d.fromDegrees(0); // Camera rotation (adjust as needed)

        Pose2d detectedRobotPose = RobotDetector.cameraPositionToFieldPose(
            opponentPos,
            currentPose,
            cameraOffset,
            cameraYaw
        );

        System.out.printf("Detected robot (field frame): x=%.2f, y=%.2f, rot=%.1f°%n",
            detectedRobotPose.getX(), detectedRobotPose.getY(), detectedRobotPose.getRotation().getDegrees());

        // Apply transformation similar to AprilTag tracking
        // Key difference: Y offset is 0 (no lateral offset)

        // Step 1: Rotate around detected robot center
        Pose2d convertedRobot2d = detectedRobotPose.rotateAround(
            detectedRobotPose.getTranslation(),
            detectedRobotPose.getRotation()
        );

        // Step 2: Apply X-only offset (6 feet forward, 0 lateral)
        Pose2d convertedRobot2d2 = convertedRobot2d.transformBy(
            new Transform2d(X_OFFSET_METERS, 0.0, new Rotation2d(0))
        );

        // Step 3: Rotate back to field frame
        Pose2d convertedRobotPose = convertedRobot2d2.rotateAround(
            detectedRobotPose.getTranslation(),
            detectedRobotPose.getRotation().times(-1)
        );

        // Step 4: Face toward the detected robot (180° rotation)
        Pose2d finalPose = new Pose2d(
            convertedRobotPose.getX(),
            convertedRobotPose.getY(),
            convertedRobotPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );

        System.out.printf("Current pose: x=%.2f, y=%.2f, rot=%.1f°%n",
            currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees());
        System.out.printf("Target pose: x=%.2f, y=%.2f, rot=%.1f°%n",
            finalPose.getX(), finalPose.getY(), finalPose.getRotation().getDegrees());

        // Calculate distance to target
        double distance = currentPose.getTranslation().getDistance(finalPose.getTranslation());
        System.out.printf("Distance to target: %.2f meters%n", distance);

        // Set speed based on slow mode
        if (isSlow) {
            speed = 0.2;
        }

        // Create path constraints
        PathConstraints constraints = new PathConstraints(
            drivebase.getSwerveDrive().getMaximumChassisVelocity() * speed,
            4.0,
            drivebase.getSwerveDrive().getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720)
        );

        System.out.printf("Max velocity: %.2f m/s, Speed multiplier: %.2f%n",
            drivebase.getSwerveDrive().getMaximumChassisVelocity(), speed);

        // Try using drivebase.driveToPose() instead of AutoBuilder
        // This should work even if PathPlanner pathfinding has issues
        System.out.println("Using drivebase.driveToPose() method");
        Command pathCommand = drivebase.driveToPose(finalPose);
        System.out.printf("Generated path command: %s%n", pathCommand.getName());
        return pathCommand;
    }
}
