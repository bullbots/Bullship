// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
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
import frc.robot.commands.Coral.ShootCoralWait;
import frc.robot.commands.Coral.YeetCoral;
import frc.robot.commands.Elevator.MoveElevatorToPos;
import frc.robot.commands.Elevator.MoveElevatorToPosWithFinish;
import frc.robot.commands.Lift.MoveLiftDown;
import frc.robot.commands.Lift.MoveLiftUp;
import frc.robot.commands.StrafeAndMoveForward;
import frc.robot.commands.swervedrive.SwervePathToAprilTagSupplier;
import frc.robot.subsystems.AlgaeExtractor;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverXbox = new CommandXboxController(0);
  public static final CommandJoystick buttonBox = new CommandJoystick(OperatorConstants.kCopilotControllerPort);

  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/falcon"));
  public static final Elevator elevator = new Elevator();

  public static final DigitalInput coralSensor = new DigitalInput(0);

  public static final Coral coral = new Coral();

  public static final Lift lift = new Lift();

  public static final AlgaeExtractor algaeExtractor = new AlgaeExtractor();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  // using opposite value until we understand
  SwerveInputStream driveStrafeRight = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> 0.03,
      () -> -0.20)
      .withControllerRotationAxis(() -> 0)
      .allianceRelativeControl(false);
  // .robotRelative(true);

  SwerveInputStream driveStrafeLeft = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> 0.03,
      () -> 0.20)
      .withControllerRotationAxis(() -> 0)
      .allianceRelativeControl(false);
  // .robotRelative(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    //NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("FirstReefRight", new DeferredCommand(new SwervePathToAprilTagSupplier(1.0), Set.of(drivebase)));
    NamedCommands.registerCommand("GoToFourthLevel", new MoveElevatorToPosWithFinish(elevator, 3, driveAngularVelocity));
    NamedCommands.registerCommand("YeetCoral", new ShootCoralWait(0.5, coral, coralSensor).andThen(new YeetCoral(coral, coralSensor)));
    NamedCommands.registerCommand("GoToBottomLevel", new MoveElevatorToPosWithFinish(elevator, 0, driveAngularVelocity));
    NamedCommands.registerCommand("RunIntake", new IntakeCoral(coral, coralSensor));
    NamedCommands.registerCommand("FirstReefLeft", new DeferredCommand(new SwervePathToAprilTagSupplier(-1.0), Set.of(drivebase)));
    Autos.load();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    // Set all defaults in this section.
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
      // drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      // driverXbox.x().onTrue(new MoveElevator(elevator, 0));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));

      driverXbox.start().onTrue((Commands.print("drivebase::zeroGyro").andThen(drivebase::zeroGyroWithAlliance)));
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.rightTrigger().whileTrue(new ShootCoral(coral, coralSensor));
      driverXbox.rightBumper().whileTrue(new IntakeCoral(coral, coralSensor));
      driverXbox.leftTrigger().whileTrue(new AlgaeArmsMoveDown(algaeExtractor));
      driverXbox.leftBumper().onTrue(new AlgaeArmsMoveUp(algaeExtractor));
      driverXbox.a().whileTrue(new AlgaeArmsBarf(algaeExtractor));
      driverXbox.y().onTrue(Commands.run(()->{elevator.childSafetyEnabled = false;}));

      driverXbox.povUp().whileTrue(new MoveLiftUp(lift));
      driverXbox.povDown().whileTrue(new MoveLiftDown(lift));

      driverXbox.povRight().whileTrue(
              new ConditionalCommand(
                      new DeferredCommand(new SwervePathToAprilTagSupplier(1.0), Set.of(drivebase)),
                      new StrafeAndMoveForward(drivebase, driveStrafeRight),
                      drivebase::seesAprilTag
              )
      );

      driverXbox.povLeft().whileTrue(
              new ConditionalCommand(
                      new DeferredCommand(new SwervePathToAprilTagSupplier(-1.0), Set.of(drivebase)),
                      new StrafeAndMoveForward(drivebase, driveStrafeLeft),
                      drivebase::seesAprilTag
              )
      );

      driverXbox.b().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      // Coral levels
      setResetCommandLevelButton(0, 4);
      setResetCommandLevelButton(1, 3);
      setResetCommandLevelButton(2, 2);
      setResetCommandLevelButton(3, 1);
      // Algae levels
      setResetCommandLevelButton(4, 5);
      setResetCommandLevelButton(5, 11);
    }
  }

  private void setResetCommandLevelButton(int level, int buttonNum) {
    var command = new MoveElevatorToPos(elevator, level, driveAngularVelocity);
    buttonBox.button(buttonNum).onTrue(new InstantCommand(() -> {
      if(command.isScheduled()) {
        command.cancel();
      }
      command.schedule();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.getSelected();
    // return drivebase.getAutonomousCommand("straight_auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
