package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
public class MoveElevatorToPos extends Command{
    
    //THIS IS SET UP FOR HOLDING A BUTTON :thimbs:
    //untested
    private final Elevator m_ElevatorSubsystem;
    private int m_level;

    
    public MoveElevatorToPos(Elevator elevator, int level){
        m_ElevatorSubsystem = elevator;
        m_level = level;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_ElevatorSubsystem.moveToLevel(m_level);
        System.out.println("MoveElevator.initialize ");
    } 
    @Override
    public void execute(){
        // m_ElevatorSubsystem.StopElevator();
    }

    @Override
    public void end(boolean isFinished){
        m_ElevatorSubsystem.stopElevator();
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
