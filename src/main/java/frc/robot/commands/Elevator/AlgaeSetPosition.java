// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Algae.AlgaeArmsMoveDown;
import frc.robot.subsystems.AlgaeExtractor;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeSetPosition extends ParallelCommandGroup {
  /** Creates a new AlgeaPositioning. */
  public AlgaeSetPosition(int level, Elevator elevator, AlgaeExtractor algeaExtractor ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new MoveElevatorToPos(elevator, level, null),new AlgaeArmsMoveDown(algeaExtractor)  );
  }
}
