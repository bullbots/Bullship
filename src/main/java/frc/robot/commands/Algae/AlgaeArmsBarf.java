// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeExtractor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeArmsBarf extends Command {
  private AlgaeExtractor algeaExtractor;

  /** Creates a new AlgeaArmsMoveDown. */
  public AlgaeArmsBarf(AlgaeExtractor algeaExtractor) {
    this.algeaExtractor = algeaExtractor;
    addRequirements(algeaExtractor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algeaExtractor.barf();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      algeaExtractor.moveArmsHold();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
