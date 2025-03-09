package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Elevator extends SubsystemBase{
    //private final TalonFX m_LeftLiftMotor;
    //private final TalonFX m_RightLiftMotor;
    //private final CANBus kCANBus = new CANBus("canivore");
    private final TalonFX m_elevatorMotor;
    private MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    boolean locked = false;
    int currentLevel = 1;
    public Elevator(){
        m_elevatorMotor = new TalonFX(Constants.Motors.ELEVATOR);
        //set up and configure the motors
        configElevatorMotors();
    }
    //The motion magic settings are just copeid from last year so may need to be adjusted, and should be tested
    public void configElevatorMotors(){
        //TalonFXConfiguration config = new TalonFXConfiguration();
        var  elevatorConfiguration = new TalonFXConfiguration();
        
        elevatorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        //elevator.setSafetyEnabled(true);

        /* Configure current limits */
        MotionMagicConfigs mm =  elevatorConfiguration.MotionMagic;
        mm.MotionMagicCruiseVelocity = 25; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 50; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 =  elevatorConfiguration.Slot0;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kV = 0.12;
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb =  elevatorConfiguration.Feedback;
        fdb.SensorToMechanismRatio = 12.8;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status =  m_elevatorMotor.getConfigurator().apply(elevatorConfiguration);
            
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status);
        }
    }


    public void MoveElevatorUp(){
        if(locked){
            System.out.println("elevator locked");
            return;
        }
        m_elevatorMotor.set(0.25);
    }
    public void MoveElevatorDown(){
        if(locked){
            System.out.println("elevator locked");
            return;
        }
        m_elevatorMotor.set(-1);
    }
    public void StopElevator(){
        m_elevatorMotor.set(0);  
    }
    
    public void LockElevator(){
        locked = true;
    }
    public void UnlockElevator(){
        locked = false;
    }
    public void MoveToLevel(int level){
        
        System.out.println("Elevator.MoveToLevel()");
        m_elevatorMotor.setControl(m_mmReq.withPosition(Constants.ElevatorLevelOffsets[level]).withSlot(0));
        m_elevatorMotor.setControl(m_mmReq.withPosition(2.97).withSlot(0));
    }

    @Override
    public void periodic() {
        var elevatorEncoder = m_elevatorMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder);
        
    }
    



}
