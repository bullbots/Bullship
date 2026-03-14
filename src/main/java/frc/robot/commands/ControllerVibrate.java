package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ControllerVibrate extends WaitCommand {
    public ControllerVibrate(double seconds) {
        super(seconds);
    }

    @Override
    public void initialize() {
        super.initialize();
        RobotContainer.driverXbox.getHID().setRumble(RumbleType.kLeftRumble, 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        RobotContainer.driverXbox.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
    }
}