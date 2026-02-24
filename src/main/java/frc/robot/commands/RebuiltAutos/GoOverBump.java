package frc.robot.commands.RebuiltAutos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class GoOverBump extends WaitCommand {
    int wait;
  final int stop = 0;
  SwerveSubsystem swerveSubsystem;

  public GoOverBump(double seconds, SwerveSubsystem ss) {
    super(seconds);
    swerveSubsystem = ss;
    addRequirements(ss);
}
@Override
  public void initialize() {
    super.initialize();
    wait = 0;
    System.out.println("GoOverBump initialize");
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var direction = 1.0;

    if (swerveSubsystem.isRedAlliance()) {

      direction = 1.0;
    }

    swerveSubsystem.driveFieldOriented(new ChassisSpeeds(-2 * direction, 0, 0));
    wait += 1;
  }
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     swerveSubsystem.driveFieldOriented(new ChassisSpeeds());
     System.out.println("Ending GoOverBump");
   }
 }
