package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Coral;

public class ShootCoralWait extends WaitCommand {

    private final Coral m_Coral;

    // for holding a button
    public ShootCoralWait(double waitTime, Coral coral, DigitalInput sensor) {
        super(waitTime);
        m_Coral = coral;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_Coral.moveCoral();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean isFinished) {
        super.end(isFinished);
        m_Coral.stopCoral();
    }

    // if not working switch this to !m_Sensor.get()
    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}