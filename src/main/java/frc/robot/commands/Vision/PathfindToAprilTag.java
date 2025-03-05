package frc.robot.commands.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

// This is currently not working and not being used

public class PathfindToAprilTag extends Command{

    private SwerveSubsystem drivebase;
    private Command autobuilderCommand;

    public PathfindToAprilTag(SwerveSubsystem swerveSubsystem) {

        drivebase = swerveSubsystem;
    }

    @Override
    public void initialize() {

        //i'm not sure if this is the best way to do this but it should work
        // System.out.println("x pressed()");
        if(drivebase.seesAprilTag()){
          
          // var cur_pose = drivebase.getPose();
          // System.out.printf("pose: %s%n", cur_pose);
          var poseEstimate = drivebase.getBlueBotPoseEstimate();
          // drivebase.resetOdometry(poseEstimate);
          // cur_pose = drivebase.getPose();
          System.out.printf("poseEstimate: %s%n", poseEstimate);

          PathConstraints constraints = new PathConstraints(
          drivebase.getSwerveDrive().getMaximumChassisVelocity()*.25, 4.0,
          drivebase.getSwerveDrive().getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
          
          // var targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
          // var results = LimelightHelpers.getLatestResults("limelight");
          // var fiducials = results.targets_Classifier;

          var id = LimelightHelpers.getFiducialID("limelight");

          var aprilTagPose = drivebase.getAprilTagPose((int) id);

          Pose2d aprilTag2d = new Pose2d(aprilTagPose.get().getX(), aprilTagPose.get().getY(), aprilTagPose.get().getRotation().toRotation2d());
          
          Pose2d convertedTag2d = aprilTag2d.rotateAround(aprilTag2d.getTranslation(), aprilTag2d.getRotation());

          Pose2d convertedTag2d2 = convertedTag2d.transformBy(new Transform2d(0.356, 0, new Rotation2d(0)));

          Pose2d finalPose = convertedTag2d2.rotateAround(aprilTag2d.getTranslation(), aprilTag2d.getRotation().times(-1));


          // Rotation2d aprilTagRotation = aprilTagPose.get().getRotation().toRotation2d();
          // var aprilTagVector = aprilTagPose.

          Pose2d pose = new Pose2d(finalPose.getX(), finalPose.getY(), finalPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

          // var pose2d = fiducials[0].getTargetPose_RobotSpace2D();
          // var poseRot = pose2d.rotateBy(poseEstimate.getRotation().times(-1));
          // System.out.printf("target x: %f target z: %f%n", pose2d.getX(), pose2d.getY());  
          // Pose2d pose = poseEstimate.plus(new Transform2d(poseRot.getX(), poseRot.getY(), Rotation2d.fromDegrees(60)));
          autobuilderCommand =  AutoBuilder.pathfindToPose(pose, constraints);
        }
        else{
          System.out.println("no AprilTag Visible");
        }

    }

    // @Override
    // public void end(boolean interrupted) {
    //     return 
    // }

}