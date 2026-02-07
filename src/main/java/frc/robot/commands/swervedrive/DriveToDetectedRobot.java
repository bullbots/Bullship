// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotDetector;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Direct drive command to approach a detected robot without PathPlanner.
 * Uses simple PID control to drive toward the target position.
 */
public class DriveToDetectedRobot extends Command {
    private final SwerveSubsystem drivebase;
    private final RobotDetector robotDetector;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private static final double X_OFFSET_METERS = 1.8288; // 6 feet
    private static final double POSITION_TOLERANCE = 0.1; // 10cm
    private static final double ROTATION_TOLERANCE = 5.0; // 5 degrees

    public DriveToDetectedRobot(SwerveSubsystem drivebase, RobotDetector robotDetector) {
        this.drivebase = drivebase;
        this.robotDetector = robotDetector;

        // Create PID controllers for X, Y, and rotation
        xController = new PIDController(2.0, 0.0, 0.0);
        yController = new PIDController(2.0, 0.0, 0.0);
        rotationController = new PIDController(3.0, 0.0, 0.0);

        rotationController.enableContinuousInput(-180, 180);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        System.out.println("DriveToDetectedRobot: Command started");
    }

    @Override
    public void execute() {
        // Check if we have a valid detection
        if (!robotDetector.isHealthy() || !robotDetector.hasRobots()) {
            drivebase.drive(new ChassisSpeeds(0, 0, 0));
            return;
        }

        double[] opponentPos = robotDetector.getClosestOpponentRobotCameraPosition();
        if (opponentPos == null) {
            drivebase.drive(new ChassisSpeeds(0, 0, 0));
            return;
        }

        // Get current pose
        Pose2d currentPose = drivebase.getPose();

        // Convert camera position to field coordinates
        Translation2d cameraOffset = new Translation2d(
            Units.inchesToMeters(14.5),
            Units.inchesToMeters(0)
        );
        Rotation2d cameraYaw = Rotation2d.fromDegrees(0);

        Pose2d detectedRobotPose = RobotDetector.cameraPositionToFieldPose(
            opponentPos,
            currentPose,
            cameraOffset,
            cameraYaw
        );

        // Calculate target pose (6 feet in front of detected robot)
        Pose2d convertedRobot2d = detectedRobotPose.rotateAround(
            detectedRobotPose.getTranslation(),
            detectedRobotPose.getRotation()
        );

        Pose2d convertedRobot2d2 = convertedRobot2d.transformBy(
            new Transform2d(X_OFFSET_METERS, 0.0, new Rotation2d(0))
        );

        Pose2d convertedRobotPose = convertedRobot2d2.rotateAround(
            detectedRobotPose.getTranslation(),
            detectedRobotPose.getRotation().times(-1)
        );

        Pose2d targetPose = new Pose2d(
            convertedRobotPose.getX(),
            convertedRobotPose.getY(),
            convertedRobotPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );

        // Calculate errors
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

        // Calculate velocities using PID
        double xVelocity = xController.calculate(0, -xError); // Negate to move toward target
        double yVelocity = yController.calculate(0, -yError);
        double rotationVelocity = rotationController.calculate(0, -rotationError);

        // Clamp velocities
        double maxVel = 1.2; // m/s (slow for testing)
        xVelocity = Math.max(-maxVel, Math.min(maxVel, xVelocity));
        yVelocity = Math.max(-maxVel, Math.min(maxVel, yVelocity));
        rotationVelocity = Math.max(-2.0, Math.min(2.0, rotationVelocity)); // rad/s

        // Create field-relative chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity,
            yVelocity,
            rotationVelocity,
            currentPose.getRotation()
        );

        drivebase.drive(speeds);

        // Debug output every 25 iterations (~0.5 seconds)
        if (Math.random() < 0.02) {
            System.out.printf("Target: (%.2f, %.2f, %.1f°) Current: (%.2f, %.2f, %.1f°) Error: (%.2f, %.2f, %.1f°)%n",
                targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees(),
                currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees(),
                xError, yError, rotationError);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new ChassisSpeeds(0, 0, 0));
        System.out.println("DriveToDetectedRobot: Command ended");
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
