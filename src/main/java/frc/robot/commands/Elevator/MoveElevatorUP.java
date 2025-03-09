// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorUP extends Command {
  private Elevator m_Elevator;
  /** Creates a new MoveElevatorUP. */
  public MoveElevatorUP(Elevator elevator) {
    addRequirements(elevator);
    m_Elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elevator.MoveElevatorUp();
    System.out.println("MoveElevatorUP.initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.StopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
