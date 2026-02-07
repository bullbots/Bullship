// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shake extends Command {

  private final SwerveSubsystem swerve;
  private final Timer timer;
  private final double shakeSpeed = Math.PI * 10; // Rotation speed in rad/s
  private final double shakePeriod = 0.5; // Time in seconds for each direction
  private boolean clockwise = true;

  /** Creates a new shake. */
  public Shake(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    clockwise = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Switch direction every shakePeriod seconds
    if (timer.hasElapsed(shakePeriod)) {
      clockwise = !clockwise;
      timer.restart();
    }

    // Apply rotation in current direction
    double rotationSpeed = clockwise ? shakeSpeed : -shakeSpeed;
    swerve.drive(new ChassisSpeeds(0, 0, rotationSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
