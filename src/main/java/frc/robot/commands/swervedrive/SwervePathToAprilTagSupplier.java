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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

/** Add your docs here. */
public class SwervePathToAprilTagSupplier implements Supplier<Command>{
    
    private double direction;
    private boolean isSlow;
    private boolean shouldCheck;
    private double speed =.35;
    public SwervePathToAprilTagSupplier(double direction, boolean isSlow, boolean shouldCheck){
        this.direction = direction;
        this.isSlow = isSlow;
        this.shouldCheck = shouldCheck;

    } 
    @Override
    public Command get() {
        System.out.printf("SwervePathToAprilTagSupplier.get() %f%n", direction);

        var drivebase = RobotContainer.drivebase;
        if(drivebase.seesAprilTag()){
          
            // var cur_pose = drivebase.getPose();
            // System.out.printf("pose: %s%n", cur_pose);
            var poseEstimate = drivebase.getBlueBotPoseEstimate();
            // drivebase.resetOdometry(poseEstimate);
            // cur_pose = drivebase.getPose();
            // System.out.printf("poseEstimate: %s%n", poseEstimate);
            if(isSlow){
                speed = .2;
            }
            PathConstraints constraints;
            constraints = new PathConstraints(
                    drivebase.getSwerveDrive().getMaximumChassisVelocity()*speed, 4.0,
                    drivebase.getSwerveDrive().getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

            // Get the best AprilTag detection from PhotonVision cameras
            // Use getAllUnreadResults() for PhotonVision 2026 API
            PhotonPipelineResult bestResult = null;
            int id = -1;
            for (Cameras camera : Cameras.values()) {
                var results = camera.camera.getAllUnreadResults();
                if (!results.isEmpty()) {
                    var result = results.get(results.size() - 1); // Get most recent result
                    if (result.hasTargets()) {
                        bestResult = result;
                        id = result.getBestTarget().getFiducialId();
                        break;
                    }
                }
            }

            if (bestResult == null || id == -1) {
                return new PrintCommand("No AprilTag ID found from PhotonVision!");
            }

            // Get AprilTag position relative to robot from PhotonVision
            // This eliminates dependency on field layout - works for any AprilTag ID
            PhotonTrackedTarget bestTarget = bestResult.getBestTarget();
            Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();

            // Get tag position in robot's reference frame (just X, Y position)
            double tagX = cameraToTarget.getX();
            double tagY = cameraToTarget.getY();

            // Calculate angle from robot to tag
            Rotation2d angleToTag = new Rotation2d(tagX, tagY);

            // Calculate target position: 0.406m back from tag along the line to the tag
            // Plus optional lateral offset perpendicular to that line
            double targetDistance = Math.sqrt(tagX * tagX + tagY * tagY) - 0.406;
            double targetX = targetDistance * angleToTag.getCos();
            double targetY = targetDistance * angleToTag.getSin() + direction * 0.167;

            // Robot should face toward the tag from the target position
            // Calculate angle from target position to tag
            double deltaX = tagX - targetX;
            double deltaY = tagY - targetY;
            Rotation2d targetHeading = new Rotation2d(deltaX, deltaY);

            // Transform target position from robot-relative to field coordinates
            Transform2d robotToTarget = new Transform2d(targetX, targetY, targetHeading);
            Pose2d finalPose = poseEstimate.transformBy(robotToTarget);

            // Log rotation data for debugging alignment
            double targetRotationDegrees = finalPose.getRotation().getDegrees();
            double currentRotationDegrees = poseEstimate.getRotation().getDegrees();
            drivebase.logRotationData(targetRotationDegrees, currentRotationDegrees);

            System.out.printf("AprilTag ID %d: Target rotation: %.2f°, Current rotation: %.2f°, Diff: %.2f°%n",
                id, targetRotationDegrees, currentRotationDegrees, targetRotationDegrees - currentRotationDegrees);

            // var pose2d = fiducials[0].getTargetPose_RobotSpace2D();
            // var poseRot = pose2d.rotateBy(poseEstimate.getRotation().times(-1));
            // System.out.printf("target x: %f target z: %f%n", pose2d.getX(), pose2d.getY());
            // Pose2d pose = poseEstimate.plus(new Transform2d(poseRot.getX(), poseRot.getY(), Rotation2d.fromDegrees(60)));
            return AutoBuilder.pathfindToPose(finalPose, constraints);
          
    
        }
        return new PrintCommand("no april tag !!");
    }
}


