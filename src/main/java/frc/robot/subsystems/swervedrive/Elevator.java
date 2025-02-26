package frc.robot.subsystems.swervedrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Elevator extends SubsystemBase{
    private final TalonFX m_LeftElevatorMotor;
    private final TalonFX m_RightElevatorMotor;

    
    boolean locked = false;
    private static Elevator instance = null;
    public Elevator(){
        m_LeftElevatorMotor = new TalonFX(Constants.Motors.ELEVATOR_LEFT);
        m_RightElevatorMotor = new TalonFX(Constants.Motors.ELEVATOR_RIGHT);
        //set up and configure the motors
    }
    public void configElevatorMotors(){
        TalonFXConfiguration l_config = new TalonFXConfiguration();
        TalonFXConfiguration r_config = new TalonFXConfiguration();

        //idk what to put here but there should be stuff here
        //the slider uses motionmagic to config, along with other stuff as seen in Twenty20Forte
    }
    public Elevator getInstance(){
        if(instance == null){
            instance = new Elevator();
        }
        return instance;
    }


    //THESE DIRECTIONS ARE ALL GUESSED AND UNTESTED
    public void MoveElevatorUp(){
        if(locked){
            System.out.println("elevator locked");
            m_LeftElevatorMotor.set(1);
            m_RightElevatorMotor.set(1);

            return;
        }
    }
    public void MoveElevatorDown(){
        if(locked){
            System.out.println("elevator locked");
            m_LeftElevatorMotor.set(-1);
            m_RightElevatorMotor.set(-1);
            return;
        }
    }
    public void StopElevator(){
        m_LeftElevatorMotor.set(0);
        m_RightElevatorMotor.set(0);    }

    



}
