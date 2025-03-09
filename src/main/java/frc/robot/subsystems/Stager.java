package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Stager extends SubsystemBase {

    public enum Mode {
        MAX_SPEED,
        HALF_SPEED,
        IN_BETWEEN,
        QUARTER_SPEED,
        BURRITO
    }


    //Staging uses two different motors
    TalonSRX m_StagerMotor;

    public Stager() {
        //TalonSRX requires Phoenix 5
        //m_StagerMotor = new TalonSRX(Constants.Motors.STAGER);
    }


    public void start(Mode mode) {

        switch (mode) {
            case MAX_SPEED:

                m_StagerMotor.set(TalonSRXControlMode.PercentOutput, 1);
                break;
            case HALF_SPEED:

                m_StagerMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
                break;
            case IN_BETWEEN:

                m_StagerMotor.set(TalonSRXControlMode.PercentOutput, 0.4);
                break;
            case QUARTER_SPEED:

                m_StagerMotor.set(TalonSRXControlMode.PercentOutput, 0.25);
                break;

            case BURRITO:

                m_StagerMotor.set(TalonSRXControlMode.PercentOutput, -1);
                break;
            default:
                break;
        }
    }

    public void stop() {
        m_StagerMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

}
