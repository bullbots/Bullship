// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeExtractor extends SubsystemBase {
  private final SparkMax algeaMotor;
  private final RelativeEncoder encoder;
  private SparkClosedLoopController closedLoopController;

  public boolean armsOut = false;

  /** Creates a new AlgeaExtractor. */
  public AlgaeExtractor() {
    algeaMotor = new SparkMax(Constants.Motors.ALGAE_MOTOR, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    algeaMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = algeaMotor.getClosedLoopController();
    encoder = algeaMotor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    config.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    algeaMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void moveArmsDown() {
    closedLoopController.setReference(Constants.AlgeaSetPosition, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
    armsOut = true;

  }

  public void moveArmsUp() {
    closedLoopController.setReference(0, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);

    armsOut = false;

  }

  public void barf() {
    closedLoopController.setReference(Constants.AlgeaBarfPosition, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);

    armsOut = true;

  }

  public void moveArmsHold() {
    closedLoopController.setReference(Constants.AlgeaHoldPosition, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);

    armsOut = true;
  }

  @Override
  public void periodic() {
    var pos = encoder.getPosition();

    SmartDashboard.putNumber("AlgeaMotorPos", pos);
    // This method will be called once per scheduler run
  }
}
