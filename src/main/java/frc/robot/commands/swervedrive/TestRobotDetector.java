// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotDetector;

/**
 * Test command to verify RobotDetector is receiving data from NetworkTables.
 * Prints debug information when executed.
 */
public class TestRobotDetector extends Command {
    private final RobotDetector robotDetector;
    private int executeCount = 0;

    public TestRobotDetector(RobotDetector robotDetector) {
        this.robotDetector = robotDetector;
        addRequirements(robotDetector);
    }

    @Override
    public void initialize() {
        System.out.println("TestRobotDetector command started");
        executeCount = 0;
    }

    @Override
    public void execute() {
        executeCount++;
        if (executeCount % 25 == 0) { // Print every ~0.5 seconds (assuming 50Hz)
            robotDetector.printDebugInfo();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TestRobotDetector command ended");
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
