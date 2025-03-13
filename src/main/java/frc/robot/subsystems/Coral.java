package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Coral extends SubsystemBase{
    //private final TalonFX m_LeftLiftMotor;
    //private final TalonFX m_RightLiftMotor;
    //private final CANBus kCANBus = new CANBus("canivore");
    private final SparkMax coralMotor;
    
    boolean locked = false;
    int currentLevel = 1;
    private static Coral instance = null;
    public Coral(){
        coralMotor = new SparkMax(Constants.Motors.CORAL_MOTOR, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
        coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public static Coral getInstance(){
        if(instance == null){
            instance = new Coral();
        }
        return instance;
    }
    //The inversions of the motors are untested and may need to be flipped or both be the same
    //The motion magic settings are just copeid from last year so may need to be adjusted, and should be tested


    public void moveCoral(){
        if(locked){
            System.out.println("coral locked");
            return;
        }
        coralMotor.set(1);
    }
    public void stopCoral(){
        coralMotor.set(0);
    }
    
    public void LockCoral(){
        locked = true;
    }
    public void UnlockCoral(){
        locked = false;
    }

    



}
