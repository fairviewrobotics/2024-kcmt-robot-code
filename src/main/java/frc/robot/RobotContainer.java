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
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.constants.Constants.Target;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  AimSubsystem aimSubsystem = new AimSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController primaryController = new XboxController(0);
  XboxController secondaryController = new XboxController(1);

  SendableChooser<Command> superSecretMissileTech = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("AutoSpinUp", new SpinUpCommand(Target.SPEAKER, shooterSubsystem));
    NamedCommands.registerCommand("AutoIntake", new IntakeCommand(intakeSubsystem));
    NamedCommands.registerCommand("AutoShoot", new ShootCommand(intakeSubsystem));
    NamedCommands.registerCommand("AutoAim", new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.SPEAKER));

    superSecretMissileTech = AutoBuilder.buildAutoChooser();
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

    // PRIMARY CONTROLLER
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
    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() -> swerveSubsystem.zeroGyro())
    );

    new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            new ShootCommand(intakeSubsystem)
    );




    // SECONDARY CONTROLLER

    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            new SpinUpCommand(Target.SPEAKER, shooterSubsystem)
    );

    new JoystickButton(secondaryController, XboxController.Axis.kLeftTrigger.value).whileTrue(
            new SpinUpCommand(Target.HIGH_PASS, shooterSubsystem)
    );
    new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
            new ParallelCommandGroup(
                    new RotateTo(swerveSubsystem, primaryController, Target.NOTE),
                    new IntakeCommand(intakeSubsystem)
            )
    );
    new JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
            new ParallelCommandGroup(
                    new RotateTo(swerveSubsystem, primaryController, Target.AMP),
                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.AMP),
                    new SpinUpCommand(Target.AMP, shooterSubsystem)

            )
    );
    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
            new ParallelCommandGroup(
                    new RotateTo(swerveSubsystem, primaryController, Target.HIGH_PASS),
                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.HIGH_PASS)
            )
    );
    new JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
            new ParallelCommandGroup(
                    new RotateTo(swerveSubsystem, primaryController, Target.LOW_PASS),
                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.LOW_PASS)
            )
    );
    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
            new ParallelCommandGroup(
                    new RotateTo(swerveSubsystem, primaryController, Target.SPEAKER),
                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.SPEAKER)
            )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return superSecretMissileTech.getSelected();
  }
}
