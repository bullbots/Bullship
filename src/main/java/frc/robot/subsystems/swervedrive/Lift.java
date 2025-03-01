package frc.robot.subsystems.swervedrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
public class Lift extends SubsystemBase{
    private final TalonFX m_LeftLiftMotor;
    private final TalonFX m_RightLiftMotor;


    
    boolean locked = false;
    int currentLevel = 1;
    private static Lift instance = null;
    public Lift(){
        m_LeftLiftMotor = new TalonFX(Constants.Motors.ELEVATOR_LEFT);
        m_RightLiftMotor = new TalonFX(Constants.Motors.ELEVATOR_RIGHT);
        //set up and configure the motors
    }
    public Lift getInstance(){
        if(instance == null){
            instance = new Lift();
        }
        return instance;
    }
    //The inversions of the motors are untested and may need to be flipped or both be the same
    //The motion magic settings are just copeid from last year so may need to be adjusted, and should be tested
    public void configElevatorMotors(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        /* Configure current limits */
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = 25; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 50; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kV = 0.12;
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs r_fdb = config.Feedback;
        r_fdb.SensorToMechanismRatio = 12.8;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_RightLiftMotor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device (right). Error: " + status);
        }
    }


    public void MoveElevatorUp(){
        if(locked){
            System.out.println("elevator locked");
            m_LeftLiftMotor.set(1);
            m_RightLiftMotor.set(1);

            return;
        }
    }
    public void MoveElevatorDown(){
        if(locked){
            System.out.println("elevator locked");
            m_LeftLiftMotor.set(-1);
            m_RightLiftMotor.set(-1);
            return;
        }
    }
    public void StopElevator(){
        m_LeftLiftMotor.set(0);
        m_RightLiftMotor.set(0);    
    }
    
    public void LockElevator(){
        locked = true;
    }
    public void UnlockElevator(){
        locked = false;
    }
    public void MoveToLevel(int level){
        //im not sure if this is correct
        m_LeftLiftMotor.setPosition(Constants.LiftLevelOffsets[level-1]);
        m_RightLiftMotor.setPosition(Constants.LiftLevelOffsets[level-1]);
        

    }
    



}
