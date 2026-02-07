// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Tornado extends Command {
  private final SwerveSubsystem swerve;
  private final double driveSpeed = 2.0; // meters per second forward
  private final double rotationSpeed = Math.PI * 2; // radians per second rotation

  /** Creates a new Tornado. */
  public Tornado(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Create field-oriented chassis speeds (relative to the field, not the robot)
    // Positive X = away from alliance wall, Positive Y = to the left when looking from alliance wall
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
        driveSpeed,      // vx: forward speed (m/s)
        0,               // vy: strafe speed (m/s)
        rotationSpeed    // omega: rotation speed (rad/s)
    );

    // Convert from field-relative to robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds,
        swerve.getHeading()  // Current robot heading
    );

    // Drive the robot
    swerve.drive(robotRelativeSpeeds);
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
