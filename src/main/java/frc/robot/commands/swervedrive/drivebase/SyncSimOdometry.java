// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Command to synchronize odometry with the simulated robot's true pose.
 *
 * SIMULATION ONLY: This command will do nothing on the physical robot.
 *
 * In simulation, this resets the robot's odometry to match the physics
 * engine's ground truth pose. This is useful for testing path following
 * and autonomous routines without odometry drift.
 */
public class SyncSimOdometry extends Command {
  private final SwerveSubsystem swerve;

  /**
   * Creates a new SyncSimOdometry command.
   *
   * @param swerve The swerve drive subsystem
   */
  public SyncSimOdometry(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Only execute in simulation
    if (Robot.isSimulation()) {
      var simPose = swerve.getSwerveDrive().getSimulationDriveTrainPose();
      if (simPose.isPresent()) {
        swerve.resetOdometry(simPose.get());
        System.out.println("Synced odometry to sim pose: " + simPose.get());
      } else {
        System.out.println("Warning: Simulation pose not available");
      }
    }
    // On physical robot, this command does nothing
  }

  @Override
  public boolean isFinished() {
    return true; // Instant command - runs once and finishes
  }
}
