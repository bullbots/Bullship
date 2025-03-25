package frc.robot.subsystems;

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

public class Elevator extends SubsystemBase {

    private final TalonFX elevatorMotor;
    private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);
    public boolean childSafetyEnabled = true;
    private static final double TOLERANCE = 0.1;

    boolean locked = false;
    public int currentLevel = 0;

    public Elevator() {
        elevatorMotor = new TalonFX(Constants.Motors.ELEVATOR);
        // set up and configure the motors
        configElevatorMotors();
    }

    // The motion magic settings are just copeid from last year so may need to be
    // adjusted, and should be tested
    public void configElevatorMotors() {
        // TalonFXConfiguration config = new TalonFXConfiguration();
        var config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // elevator.setSafetyEnabled(true);

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
            status = elevatorMotor.getConfigurator().apply(config);

            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status);
        }
    }

    public void moveElevatorUp() {
        if (locked) {
            System.out.println("elevator locked");
            return;
        }
        elevatorMotor.set(0.25);
    }

    public void MoveElevatorDown() {
        if (locked) {
            System.out.println("elevator locked");
            return;
        }
        elevatorMotor.set(-1);
    }

    public void stopElevator() {
        elevatorMotor.set(0);
    }

    public void lockElevator() {
        locked = true;
    }

    public void unlockElevator() {
        locked = false;
    }

    public void moveToLevel(int level) {
        System.out.println("Elevator.MoveToLevel()");
        currentLevel = level;
        elevatorMotor.setControl(mmReq.withPosition(Constants.ElevatorLevelOffsets[level]).withSlot(0));
    }

    public boolean isAtPosition(int level) {
        double elevatorPosition = Constants.ElevatorLevelOffsets[level];
        return ((elevatorPosition - TOLERANCE) <= elevatorMotor.getPosition().getValueAsDouble()) &&
           (elevatorMotor.getPosition().getValueAsDouble() <= (elevatorPosition + TOLERANCE));
    }

    @Override
    public void periodic() {
        // Remove debug print/SmartDashboard statements in main.
        var elevatorEncoder = elevatorMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder);
        // SmartDashboard.putNumber("Elevator Currant",
        // m_elevatorMotor.getSupplyCurrent().getValueAsDouble());
    }
}
