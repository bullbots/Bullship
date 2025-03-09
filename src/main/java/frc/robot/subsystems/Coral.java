package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Coral extends SubsystemBase{
    //private final TalonFX m_LeftLiftMotor;
    //private final TalonFX m_RightLiftMotor;
    //private final CANBus kCANBus = new CANBus("canivore");
    private final TalonFX coralMotor;
    
    boolean locked = false;
    int currentLevel = 1;
    private static Coral instance = null;
    public Coral(){
        coralMotor = new TalonFX(Constants.Motors.ELEVATOR);
        configCoralMotors();
        //set up and configure the motors
    }
    public static Coral getInstance(){
        if(instance == null){
            instance = new Coral();
        }
        return instance;
    }
    //The inversions of the motors are untested and may need to be flipped or both be the same
    //The motion magic settings are just copeid from last year so may need to be adjusted, and should be tested
    public void configCoralMotors(){
        //TalonFXConfiguration config = new TalonFXConfiguration();
        var coralMotorConfig = new TalonFXConfiguration();
        // coralMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        coralMotor.setSafetyEnabled(true);
        /* Configure current limits */
        MotionMagicConfigs mm = coralMotorConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = 25; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 50; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 = coralMotorConfig.Slot0;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kV = 0.12;
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs r_fdb = coralMotorConfig.Feedback;
        r_fdb.SensorToMechanismRatio = 12.8;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = coralMotor.getConfigurator().apply(coralMotorConfig);
            
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status);
        }
    }


    public void MoveCoral(){
        if(locked){
            System.out.println("coral locked");
            return;
        }
        coralMotor.set(1);
    }
    public void StopCoral(){
        coralMotor.set(0);
    }
    
    public void LockCoral(){
        locked = true;
    }
    public void UnlockCoral(){
        locked = false;
    }

    



}
