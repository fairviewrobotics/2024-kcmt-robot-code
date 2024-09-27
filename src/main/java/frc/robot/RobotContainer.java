// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.constants.Constants.Target;
import frc.robot.utils.Controller;
import org.opencv.core.Mat;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  AimSubsystem aimSubsystem = new AimSubsystem();

  // Controllers
  XboxController primaryController = new XboxController(0);
  XboxController secondaryController = new XboxController(1);

  Controller testController = new Controller(2);

  // Auto Chooser
  SendableChooser<Command> superSecretMissileTech = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Register commands for PathPlanner
//    NamedCommands.registerCommand("AutoSpinUp", new SpinUpCommand(Target.SPEAKER, shooterSubsystem).withTimeout(20));
//    NamedCommands.registerCommand("AutoIntake", new IntakeCommand(intakeSubsystem, false).withTimeout(1.5));
//    NamedCommands.registerCommand("AutoIntakeContinuous4", new IntakeCommand(intakeSubsystem, true).withTimeout(4.0));
//    NamedCommands.registerCommand("AutoIntakeContinuous1.5", new IntakeCommand(intakeSubsystem, true).withTimeout(1.5));
//    NamedCommands.registerCommand("AutoShoot", new ShootCommand(intakeSubsystem).withTimeout(0.3));
//    NamedCommands.registerCommand("AutoAim", new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.SPEAKER).withTimeout(20));

    NamedCommands.registerCommand("AutoSpinUp", new InstantCommand());
    NamedCommands.registerCommand("AutoIntake", new InstantCommand());
    NamedCommands.registerCommand("AutoIntakeContinuous4", new InstantCommand());
    NamedCommands.registerCommand("AutoIntakeContinuous1.5", new InstantCommand());
    NamedCommands.registerCommand("AutoShoot", new InstantCommand());
    NamedCommands.registerCommand("AutoAim", new InstantCommand());


    // Set up auto chooser
    superSecretMissileTech = AutoBuilder.buildAutoChooser();
    // Put the chooser on the dashboard
    SmartDashboard.putData("AutoChooser", superSecretMissileTech);

    // Configure the trigger bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    // Default commands
//    aimSubsystem.setDefaultCommand(
//            new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.NOTE)
//    );

    // PRIMARY CONTROLLER

    // Default drive command
    swerveSubsystem.setDefaultCommand(
            new DriveCommands(
                    swerveSubsystem,
                    () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar,
                    () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar,
                    () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar,
                    true,
                    true
            )
    );

    // Slow drive command: Right bumper
    new JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            new DriveCommands(
                    swerveSubsystem,
                    () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar/2.0,
                    () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar/2.0,
                    () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar/2.0,
                    true,
                    true
            )
    );

    // Zero gyro: Y button
    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() -> swerveSubsystem.zeroGyro())
    );

    // Shoot(run intake forward): Left bumper
    new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            new ShootCommand(intakeSubsystem)
    );

    // SECONDARY CONTROLLER

    // Spin up shooter fast: Left bumper
//    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
//            new SpinUpCommand(Target.SPEAKER, shooterSubsystem)
//    );
    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
      new SpinUpCommand(Target.HIGH_PASS, shooterSubsystem)
    );


    // Spin up shooter slow: Left trigger
    new JoystickButton(secondaryController, XboxController.Axis.kLeftTrigger.value).whileTrue(
            new SpinUpCommand(Target.HIGH_PASS, shooterSubsystem)
    );

    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() -> aimSubsystem.setAngle(Math.toRadians(45)))
    ).whileFalse(
            new RunCommand(() -> {
              aimSubsystem.runLeft(0.0);
              aimSubsystem.runRight(0.0);
            })
    );

    new JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
            new RunCommand(() -> aimSubsystem.resetPID())
    );

    // Intake + rotate to note: Right bumper
    new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            new ParallelCommandGroup(
//                    new RotateTo(swerveSubsystem, primaryController, Target.NOTE),
                    new IntakeCommand(intakeSubsystem, true)
//            )
    );

    new POVButton(secondaryController, 0).whileTrue(
        new RunCommand(() -> intakeSubsystem.setSpeed(-0.3))
    ).whileFalse(
        new RunCommand(() -> intakeSubsystem.setSpeed(0.0))
    );

    // Raise shooter + rotate to amp + spin up for amp: X button
//    new JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
//            new ParallelCommandGroup(
//                    new RotateTo(swerveSubsystem, primaryController, Target.AMP),
//                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.AMP),
//                    new SpinUpCommand(Target.AMP, shooterSubsystem)
//
//            )
//    );

    // Rotate to high pass + aim at high pass: Y button
//    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
//            new ParallelCommandGroup(
//                    new RotateTo(swerveSubsystem, primaryController, Target.HIGH_PASS),
//                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.HIGH_PASS)
//            )
//    );

    // Rotate to low pass + aim at low pass: B button
//    new JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
//            new ParallelCommandGroup(
//                    new RotateTo(swerveSubsystem, primaryController, Target.LOW_PASS),
//                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.LOW_PASS)
//            )
//    );

    // Rotate to speaker + aim at speaker: A button
//    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
//            new ParallelCommandGroup(
//                    new RotateTo(swerveSubsystem, primaryController, Target.SPEAKER),
//                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.SPEAKER)
//            )
//    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return superSecretMissileTech.getSelected();
  }
}
