// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Algae.AlgaeArmsBarf;
import frc.robot.commands.Algae.AlgaeArmsMoveDown;
import frc.robot.commands.Algae.AlgaeArmsMoveUp;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Coral.IntakeCoral;
import frc.robot.commands.Coral.ShootCoral;
import frc.robot.commands.Elevator.MoveElevatorToPos;
import frc.robot.commands.StrafeAndMoveForward;
import frc.robot.subsystems.AlgaeExtractor;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverXbox = new CommandXboxController(0);
  public static final CommandJoystick buttonBox = new CommandJoystick(OperatorConstants.kCopilotControllerPort);

  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve/falcon"));
  public static final Elevator elevator = new Elevator();

  public static final DigitalInput coralSensor = new DigitalInput(0);

  public static final Coral coral = new Coral();

  public static final Lift lift = new Lift();

  public static final AlgaeExtractor algaeExtractor = new AlgaeExtractor();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * 1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

//using opposite value until we understand
  SwerveInputStream driveStrafeRight = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                             () -> -0.05,
                                                             () -> 0.25)
                                                         .withControllerRotationAxis(() -> 0)
                                                         .allianceRelativeControl(false);
                                                         //.robotRelative(true);

  SwerveInputStream driveStrafeLeft = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                         () -> -0.05,
                                                         () -> -0.25)
                                                     .withControllerRotationAxis(() -> 0)
                                                     .allianceRelativeControl(false);
                                                    // .robotRelative(true);


//  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
//                                                                        () -> -driverXbox.getLeftY(),
//                                                                        () -> -driverXbox.getLeftX())
//                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
//                                                                        2))
//                                                                    .deadband(OperatorConstants.DEADBAND)
//                                                                    .scaleTranslation(0.8)
//                                                                    .allianceRelativeControl(true);
//  // Derive the heading axis with math!
//  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
//                                                                               .withControllerHeadingAxis(() ->
//                                                                                                              Math.sin(
//                                                                                                                  driverXbox.getRawAxis(
//                                                                                                                      2) *
//                                                                                                                  Math.PI) *
//                                                                                                              (Math.PI *
//                                                                                                               2),
//                                                                                                          () ->
//                                                                                                              Math.cos(
//                                                                                                                  driverXbox.getRawAxis(
//                                                                                                                      2) *
//                                                                                                                  Math.PI) *
//                                                                                                              (Math.PI *
//                                                                                                               2))
//                                                                               .headingWhile(true)
//                                                                               .translationHeadingOffset(true)
//                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
//                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    Autos.load();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
//    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
//    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
//    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
//        driveDirectAngle);
//    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
//    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
//    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
//        driveDirectAngleKeyboard);

    // Set all defaults in this section.
    if (RobotBase.isSimulation())
    {
//      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
//      driveDirectAngleKeyboard.driveToPose(() -> target,
//                                           new ProfiledPIDController(5,
//                                                                     0,
//                                                                     0,
//                                                                     new Constraints(5, 2)),
//                                           new ProfiledPIDController(5,
//                                                                     0,
//                                                                     0,
//                                                                     new Constraints(Units.degreesToRadians(360),
//                                                                                     Units.degreesToRadians(180))
//                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
//      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
//                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      //driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      //driverXbox.x().onTrue(new MoveElevator(elevator, 0));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      // driverXbox.leftBumper().onTrue(new MoveLift(lift, 1));
      // driverXbox.rightBumper().onTrue(new MoveLift(lift, 2));

      driverXbox.start().onTrue((Commands.print("drivebase::zeroGyro").andThen(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.rightTrigger().whileTrue(new ShootCoral(coral, coralSensor));
      driverXbox.rightBumper().whileTrue(new IntakeCoral(coral, coralSensor));
      driverXbox.leftTrigger().onTrue(new AlgaeArmsMoveDown(algaeExtractor));
      driverXbox.leftBumper().whileTrue(new AlgaeArmsMoveUp(algaeExtractor));
      driverXbox.povRight().whileTrue(new StrafeAndMoveForward(drivebase, driveStrafeRight));
      driverXbox.povLeft().whileTrue(new StrafeAndMoveForward(drivebase, driveStrafeLeft));
      driverXbox.a().whileTrue(new AlgaeArmsBarf(algaeExtractor));
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());

      buttonBox.button(1).onTrue(new MoveElevatorToPos(elevator,3,driveAngularVelocity));

      buttonBox.button(2).onTrue(new MoveElevatorToPos(elevator,2,driveAngularVelocity));
      buttonBox.button(3).onTrue(new MoveElevatorToPos(elevator,1,driveAngularVelocity));
      buttonBox.button(4).onTrue(new MoveElevatorToPos(elevator,0,driveAngularVelocity)); 
      //buttonBox.button(9).onTrue(new MoveElevatorToPos(elevator,5));
      //buttonBox.button(6).onTrue(new MoveLiftUp(lift));
      //buttonBox.button(7).onTrue(new MoveLiftDown(lift));
      //Algea arm positions that we guessed
      buttonBox.button(5).onTrue(new MoveElevatorToPos(elevator,4,driveAngularVelocity)); 
      buttonBox.button(11).onTrue(new MoveElevatorToPos(elevator,5,driveAngularVelocity)); 


    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Autos.getSelected();
    // return drivebase.getAutonomousCommand("test");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
