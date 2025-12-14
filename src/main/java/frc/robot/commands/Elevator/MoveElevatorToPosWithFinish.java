package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import swervelib.SwerveInputStream;

public class MoveElevatorToPosWithFinish extends Command {

    // THIS IS SET UP FOR HOLDING A BUTTON :thumbs:
    private final Elevator m_ElevatorSubsystem;
    private int m_level;

    private SwerveInputStream driveAngularVelocity;

    public MoveElevatorToPosWithFinish(Elevator elevator, int level, SwerveInputStream driveAngularVelocity) {
        m_ElevatorSubsystem = elevator;
        m_level = level;

        this.driveAngularVelocity = driveAngularVelocity;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

        System.out.println("MoveElevator.initialize ");

        if (m_ElevatorSubsystem.currentLevel== 0 && RobotContainer.coralSensor.get() && m_ElevatorSubsystem.childSafetyEnabled){
            if (RobotContainer.coralSensor.get()){
                System.out.println("no coral detected!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
            // if (!RobotContainer.algaeExtractor.armsOut){
            //     System.out.println("Arms in!!!!!!!!!!!!!!!!!!!!");
            // }
            return;
        }
        m_ElevatorSubsystem.moveToLevel(m_level);

        var scalefactor = 0.8;
        if (m_level == 3 || m_level == 2) {

            scalefactor = 0.3;

        }
        driveAngularVelocity.scaleTranslation(scalefactor);
    }

    @Override
    public void execute() {
        // m_ElevatorSubsystem.StopElevator();
    }

    @Override
    public void end(boolean isFinished) {
        // m_ElevatorSubsystem.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return m_ElevatorSubsystem.isAtPosition(m_level);
    }

}
