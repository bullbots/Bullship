package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.MapProperty;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Lift;
public class MoveLift extends Command{
    
    //THIS IS SET UP FOR HOLDING A BUTTON :thimbs:
    //untested
    private final Lift m_ElevatorSubsystem;
    private int m_level;

    
    public MoveLift(Lift elevator, int level){
        m_ElevatorSubsystem = elevator;
        m_level = level;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_ElevatorSubsystem.MoveToLevel(m_level);
    } 
    @Override
    public void execute(){

    }

    @Override
    public void end(boolean isFinished){
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
