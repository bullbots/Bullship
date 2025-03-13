package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class StrafeAndMoveForward extends Command {
    SwerveSubsystem swerveSubsystem;
    SwerveInputStream inputStream;

    public StrafeAndMoveForward(SwerveSubsystem ss, SwerveInputStream input) {
        swerveSubsystem = ss;
        inputStream = input;
        addRequirements(ss);
    }

    @Override
    public void execute() {

        swerveSubsystem.drive(inputStream);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
