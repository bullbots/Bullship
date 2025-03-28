// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // IDs
  public final class Motors {

    public static final int BACK_RIGHT_DRIVE = 1;
    public static final int BACK_RIGHT_SWERVE = 2;

    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_LEFT_SWERVE = 4;

    public static final int FRONT_RIGHT_DRIVE = 5;
    public static final int FRONT_RIGHT_SWERVE = 6;

    public static final int FRONT_LEFT_DRIVE = 7;
    public static final int FRONT_LEFT_SWERVE = 8;

    public static final int ELEVATOR = 9;
    public static final int CORAL_MOTOR = 10;
    public static final int ALGAE_MOTOR = 11;
    // motor removed
    // public static final int ALGAE_DOWN = 12;
    public static final int LIFT_MOTOR = 13;

  }

  // Coral positions and Algea positions
  // 0 bottom, 2.01 second/trough, 0.98 is algae L1, 2.44 is algae L2
  public static final double[] ElevatorLevelOffsets = { 0, 2.01, 3.58, 5.95, 0.98, 2.44 };
  public static final double AlgeaSetPosition = 8;
  public static final double AlgeaBarfPosition = 14.8;
  public static final double AlgeaHoldPosition = 6;

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.15;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
  }
}
