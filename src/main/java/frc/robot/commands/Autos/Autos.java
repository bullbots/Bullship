package frc.robot.commands.Autos;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public final class Autos {
    /**
     * Example static factory for an autonomous command.
     */
    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static final SendableChooser<Command> commandChooser = new SendableChooser<>();

    public static void load() {
        commandChooser.setDefaultOption("Do Nothing", 
            new DoNothing("Doing Nothing"));

        commandChooser.addOption("Drive Forward",
            new DriveForward(3, RobotContainer.drivebase));

        SmartDashboard.putData("Command Selected", commandChooser);
    }

    public static Command getSelected() {
        return commandChooser.getSelected();
    }
}
