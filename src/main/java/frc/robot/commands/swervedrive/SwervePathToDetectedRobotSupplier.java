// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotDetector;

/**
 * Command supplier that generates a path to track a detected opponent robot.
 * Uses the RealSense robot detector running on the coprocessor to identify
 * opponent robots and generates a dynamic path to approach them.
 *
 * Target pose is calculated along the straight line from the detected robot
 * toward our current position, at a configurable offset distance, facing
 * the detected robot (y offset = 0 in target frame).
 */
public class SwervePathToDetectedRobotSupplier implements Supplier<Command> {

    private boolean isSlow;
    private double speed = 0.35;
    private double offsetMeters;

    // Telemetry publishers for visualizing target poses
    private static final StructPublisher<Pose2d> targetPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("RobotDetector/targetPose", Pose2d.struct).publish();
    private static final StructPublisher<Pose2d> detectedRobotPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("RobotDetector/detectedRobotPose", Pose2d.struct).publish();

    /**
     * Creates a new robot tracking command supplier.
     * @param offsetMeters Distance to stop from the detected robot (along the line toward us)
     * @param isSlow if true, uses slower speed (0.2 instead of 0.35)
     */
    public SwervePathToDetectedRobotSupplier(double offsetMeters, boolean isSlow) {
        this.offsetMeters = offsetMeters;
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

        Translation2d detectedRobotTranslation = detectedRobotPose.getTranslation();
        Translation2d ourTranslation = currentPose.getTranslation();

        System.out.printf("Detected robot (field frame): x=%.2f, y=%.2f%n",
            detectedRobotTranslation.getX(), detectedRobotTranslation.getY());

        // Calculate vector from detected robot to our current position
        Translation2d robotToUs = ourTranslation.minus(detectedRobotTranslation);
        double distance = robotToUs.getNorm();

        if (distance < 0.1) {
            System.out.println("Already at detected robot position!");
            return new PrintCommand("Already at detected robot position!");
        }

        // Normalize the vector and scale by offset distance
        // This gives us a point along the straight line from detected robot toward us
        Translation2d offsetVector = robotToUs.div(distance).times(offsetMeters);
        Translation2d targetTranslation = detectedRobotTranslation.plus(offsetVector);

        // Calculate rotation to face the detected robot
        // Vector from target position to detected robot
        Translation2d targetToRobot = detectedRobotTranslation.minus(targetTranslation);
        Rotation2d facingRotation = new Rotation2d(targetToRobot.getX(), targetToRobot.getY());

        Pose2d finalPose = new Pose2d(targetTranslation, facingRotation);

        // Publish poses for field visualization
        detectedRobotPosePublisher.set(detectedRobotPose);
        targetPosePublisher.set(finalPose);

        System.out.printf("Current pose: x=%.2f, y=%.2f, rot=%.1f°%n",
            currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees());
        System.out.printf("Target pose: x=%.2f, y=%.2f, rot=%.1f°%n",
            finalPose.getX(), finalPose.getY(), finalPose.getRotation().getDegrees());

        // Calculate distance to target
        double distanceToTarget = currentPose.getTranslation().getDistance(finalPose.getTranslation());
        System.out.printf("Distance to target: %.2f meters%n", distanceToTarget);

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

        // Use AutoBuilder.pathfindToPose for pathfinding with constraints
        System.out.println("Using AutoBuilder.pathfindToPose() method");
        Command pathCommand = AutoBuilder.pathfindToPose(finalPose, constraints);
        System.out.printf("Generated path command: %s%n", pathCommand.getName());
        return pathCommand;
    }
}
