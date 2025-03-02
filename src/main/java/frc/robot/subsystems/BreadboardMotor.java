package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreadboardMotor extends SubsystemBase {

    private static final TalonFX m_motor = new TalonFX(1);

    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    public BreadboardMotor() {
        //stolen from phoenix 6 motion magic example
        TalonFXConfiguration configs = new TalonFXConfiguration();

        FeedbackConfigs fdb = configs.Feedback;
        fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

        /* Configure Motion Magic */
        MotionMagicConfigs mm = configs.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        Slot0Configs slot0 = configs.Slot0;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_motor.getConfigurator().apply(configs);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    public void driveToPosition() {
        m_motor.setControl(m_mmReq.withPosition(12).withSlot(0));
    }

    // public Command rotate(double rotations) {
    //     return 
    // }

    public void stop() {

    }
    
}
