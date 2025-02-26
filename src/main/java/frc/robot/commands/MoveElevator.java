package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.MapProperty;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Elevator;
public class MoveElevator extends Command{
    
    //THIS IS SET UP FOR HOLDING A BUTTON :thimbs:
    //also may need work, I'm basing everything off of the way we did it last year last year's
    private final Elevator m_ElevatorSubsystem;
    private boolean m_moveUp;

    
    public MoveElevator(Elevator elevator, boolean moveUp){
        m_ElevatorSubsystem = elevator;
        m_moveUp = moveUp;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        if (m_moveUp){
            m_ElevatorSubsystem.MoveElevatorUp();
        }
        else{
            m_ElevatorSubsystem.MoveElevatorDown();
        }
    }
    @Override
    public void execute(){

    }

    @Override
    public void end(boolean isFinished){
        m_ElevatorSubsystem.StopElevator();
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
