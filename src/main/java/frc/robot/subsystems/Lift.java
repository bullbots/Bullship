package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {

    private final TalonFX m_liftMotor;

    boolean locked = false;
    int currentLevel = 1;

    public Lift() {
        m_liftMotor = new TalonFX(Constants.Motors.LIFT_MOTOR);
        // set up and configure the motors
        configLiftMotors();
    }

    // The motion magic settings are just copeid from last year so may need to be
    // adjusted, and should be tested
    public void configLiftMotors() {
        // TalonFXConfiguration config = new TalonFXConfiguration();
        var config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // lift.setSafetyEnabled(true);

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

        FeedbackConfigs fdb = config.Feedback;
        fdb.SensorToMechanismRatio = 12.8;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_liftMotor.getConfigurator().apply(config);

            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status);
        }

    }

    public void MoveLiftUp() {
        if (locked) {
            System.out.println("lift locked");
            return;
        }
        m_liftMotor.set(-0.7);
    }

    public void MoveLiftDown() {
        if (locked) {
            System.out.println("lift locked");
            return;
        }
        m_liftMotor.set(0.7);
    }

    public void stopLift() {
        m_liftMotor.set(0);
    }

    public void lockLift() {
        locked = true;
    }

    public void unlockLift() {
        locked = false;
    }

    @Override
    public void periodic() {
        // Remove debug print/SmartDashboard statements in main.
        var liftEncoder = m_liftMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Lift Encoder", liftEncoder);
        SmartDashboard.putNumber("Lift Current", m_liftMotor.getSupplyCurrent().getValueAsDouble());
    }
}
