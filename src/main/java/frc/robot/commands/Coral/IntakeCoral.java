package frc.robot.commands.Coral;

import com.fasterxml.jackson.databind.ser.std.MapProperty;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

public class IntakeCoral extends Command{
    
    private final Coral m_Coral;
    private DigitalInput m_Sensor;

    
    public IntakeCoral(Coral coral, DigitalInput sensor){
        m_Coral = coral;
        m_Sensor = sensor;
    }

    @Override
    public void initialize() {
        m_Coral.MoveCoral();
    } 
    @Override
    public void execute(){

    }

    @Override
    public void end(boolean isFinished){
        m_Coral.StopCoral();
    }
    //if not working switch this to !m_Sensor.get()
    @Override
    public boolean isFinished(){
        return !m_Sensor.get();
    }

}