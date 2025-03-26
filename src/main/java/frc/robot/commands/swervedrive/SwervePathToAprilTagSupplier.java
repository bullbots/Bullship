// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.lang.reflect.Constructor;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class SwervePathToAprilTagSupplier implements Supplier<Command>{
    
    private double direction;
    public SwervePathToAprilTagSupplier(double direction){
        this.direction = direction;

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

          
            PathConstraints constraints;
            constraints = new PathConstraints(
                    drivebase.getSwerveDrive().getMaximumChassisVelocity()*.50, 4.0,
                    drivebase.getSwerveDrive().getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
            
            // var targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
            // var results = LimelightHelpers.getLatestResults("limelight");
            // var fiducials = results.targets_Classifier;
          
            var id = LimelightHelpers.getFiducialID("limelight-aprilta");
          
            var aprilTagPose = drivebase.getAprilTagPose((int) id);
          
            Pose2d aprilTag2d = new Pose2d(aprilTagPose.get().getX(), aprilTagPose.get().getY(), aprilTagPose.get().getRotation().toRotation2d());
            
            Pose2d convertedTag2d = aprilTag2d.rotateAround(aprilTag2d.getTranslation(), aprilTag2d.getRotation());
          
            Pose2d convertedTag2d2 = convertedTag2d.transformBy(new Transform2d(0.406, direction*0.167, new Rotation2d(0)));
          
            Pose2d convertedTagPose = convertedTag2d2.rotateAround(aprilTag2d.getTranslation(), aprilTag2d.getRotation().times(-1));
          
            // Rotation2d aprilTagRotation = aprilTagPose.get().getRotation().toRotation2d();
            // var aprilTagVector = aprilTagPose.
          
            Pose2d finalPose = new Pose2d(convertedTagPose.getX(), convertedTagPose.getY(), convertedTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
          
            // var pose2d = fiducials[0].getTargetPose_RobotSpace2D();
            // var poseRot = pose2d.rotateBy(poseEstimate.getRotation().times(-1));
            // System.out.printf("target x: %f target z: %f%n", pose2d.getX(), pose2d.getY());  
            // Pose2d pose = poseEstimate.plus(new Transform2d(poseRot.getX(), poseRot.getY(), Rotation2d.fromDegrees(60)));
            return AutoBuilder.pathfindToPose(finalPose, constraints);
          
    
        }
        return new PrintCommand("no april tag !!");
    }
}


