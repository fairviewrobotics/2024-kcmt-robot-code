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
import frc.robot.constants.AimConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.*;
import frc.robot.constants.Constants.Target;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.Controller;
import frc.robot.utils.NetworkTableUtils;
import org.opencv.core.Mat;

import javax.xml.crypto.dsig.XMLObject;

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
  LEDSubsystem ledSubsystem = new LEDSubsystem();

  // Controllers
  XboxController primaryController = new XboxController(0);
  XboxController secondaryController = new XboxController(1);

  Controller testController = new Controller(2);

  private final NetworkTableUtils NTTune = new NetworkTableUtils("Tune");

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
                    () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar/1.0,
                    () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar/1.0,
                    () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar/1.0,
                    true,
                    true
            )
    );



    aimSubsystem.setDefaultCommand(
            new AimCommand(
                    aimSubsystem,
                    swerveSubsystem,
                    primaryController,
                    Target.DEFAULT
            )
    );

    // Slow drive command: Right bumper
    new JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            new DriveCommands(
                    swerveSubsystem,
                    () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar,
                    () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar,
                    () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar,
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
            new ShootCommand(intakeSubsystem, ledSubsystem)
    );

    new JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
            new RunCommand(() -> {
              ledSubsystem.setAnimation(LEDSubsystem.AnimationTypes.GreenStrobe);
            })
    );

    new JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            new RunCommand(() -> {
              ledSubsystem.setAnimation(LEDSubsystem.AnimationTypes.Off);
            })
    );

    // SECONDARY CONTROLLER

    // Spin up shooter fast: Left bumper
//    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
//            new SpinUpCommand(Target.SPEAKER, shooterSubsystem)
//    );
    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            new SpinUpCommand(shooterSubsystem, Target.SPEAKER)
    );


    // Spin up shooter slow: Left trigger
    new JoystickButton(secondaryController, XboxController.Axis.kLeftTrigger.value).whileTrue(
            new SpinUpCommand(shooterSubsystem, Target.HIGH_PASS)
    );

//    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
//            new RunCommand(() -> {
//
//              aimSubsystem.setAngle(Math.toRadians(45));
//            }, aimSubsystem)
//    );

//    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
//            new AimCommand(aimSubsystem, swerveSubsystem, primaryController, Target.SPEAKER)
//    );

    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() -> intakeSubsystem.setNTSpeed(), intakeSubsystem)
    ).whileFalse(
            new RunCommand((() -> intakeSubsystem.setSpeed(0)))
    );

//    ).whileFalse(
//            new RunCommand(() -> {
//              aimSubsystem.runLeft(0.0);
//              aimSubsystem.runRight(0.0);
//            })
//    );

//    new JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
//            new RunCommand(() -> aimSubsystem.resetPID())
//    );

    // Intake + rotate to note: Right bumper
    new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            new ParallelCommandGroup(
//                    new RotateTo(swerveSubsystem, primaryController, Target.NOTE),
            new IntakeCommand(intakeSubsystem, ledSubsystem, false)
//            )
    );

    new JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
            new RotateTo(swerveSubsystem, primaryController, Target.NOTE)
    );

    new POVButton(secondaryController, 0).whileTrue(
            new RunCommand(() -> intakeSubsystem.setSpeed(-0.3))
    ).whileFalse(
            new RunCommand(() -> intakeSubsystem.setSpeed(0.0))
    );

    new POVButton(secondaryController, 180).whileTrue(
            new RunCommand(() -> {
              aimSubsystem.resetEncoder();
              aimSubsystem.resetPID();
            })
    );

    new POVButton(secondaryController, 90).whileTrue(
            new RunCommand(() -> ConfigManager.getInstance().saveDefault())
    );

    new POVButton(secondaryController, 90).whileTrue(
            new RunCommand(() -> ConfigManager.getInstance().saveDefault())
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
    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
//            new ParallelCommandGroup(
//                    new RotateTo(swerveSubsystem, primaryController, Target.SPEAKER),
//                    new AimCommand(aimSubsystem, swerveSubsystem, secondaryController, Target.SPEAKER)
//            )
            new RunCommand(() -> aimSubsystem.setAngle(Math.toRadians(30)), aimSubsystem)
    );

    new JoystickButton(secondaryController, XboxController.Button.kRightStick.value).whileTrue(
            new ParallelCommandGroup(
                    new AimCommand(aimSubsystem, swerveSubsystem, primaryController, Target.AMP),
                    new SpinUpCommand(shooterSubsystem, Target.AMP)
            )
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return superSecretMissileTech.getSelected();
  }

  public void robotInit() {
  }

  public void enableInit() {
    NTTune.setDouble("intake_notein_speed", ConfigManager.getInstance().get("intake_notein_speed", Double.class, 0.0));

    aimSubsystem.resetPID();
    ledSubsystem.setAnimation(LEDSubsystem.AnimationTypes.Off);
  }

  public void disableInit() {
    ledSubsystem.setAnimation(LEDSubsystem.AnimationTypes.Rainbow);

//    ConfigManager.getInstance().setDouble("intake_notein_speed", NTTune.getDouble("intake_notein_speed", 0));
    ConfigManager.getInstance().saveConfig();
  }
}