package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

public class ShootCoral extends Command {

    private final Coral m_Coral;

    // for holding a button
    public ShootCoral(Coral coral, DigitalInput sensor) {
        m_Coral = coral;
    }

    @Override
    public void initialize() {
        m_Coral.moveCoral();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean isFinished) {
        m_Coral.stopCoral();
    }

    // if not working switch this to !m_Sensor.get()
    @Override
    public boolean isFinished() {
        return false;
    }

}