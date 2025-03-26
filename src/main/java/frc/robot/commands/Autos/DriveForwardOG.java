// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveForwardOG extends WaitCommand {
  int wait;
  final int stop = 0;
  SwerveSubsystem swerveSubsystem;

  public DriveForwardOG(double seconds, SwerveSubsystem ss) {
    super(seconds);
    swerveSubsystem = ss;
    addRequirements(ss);

  }

  @Override
  public void initialize() {
    super.initialize();
    wait = 0;
    swerveSubsystem.zeroGyroAgainstAlliance();
    System.out.println("DriveForward initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var direction = -1.0;

    if (swerveSubsystem.isRedAlliance()) {

      direction = 1.0;
    }

    swerveSubsystem.driveFieldOriented(new ChassisSpeeds(-0.5 * direction, 0, 0));
    wait += 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.driveFieldOriented(new ChassisSpeeds());
    System.out.println("Ending Drive Forward");
  }
}
