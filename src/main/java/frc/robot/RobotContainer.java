// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AimConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.constants.Constants.Target;
import frc.robot.utils.Controller;
import frc.robot.utils.NetworkTableUtils;
import org.opencv.core.Mat;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final AimSubsystem aimSubsystem = new AimSubsystem();

  // Controllers
  private final XboxController primaryController = new XboxController(0);
  private final XboxController secondaryController = new XboxController(1);

  private final NetworkTableUtils NTModeInfo = new NetworkTableUtils("Mode Info");

  private Constants.Mode mode;

  Controller testController = new Controller(2);

  // Auto Chooser
  private final SendableChooser<Command> superSecretMissileTech;

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


    swerveSubsystem.setDefaultCommand(
            new DriveCommands(
                    swerveSubsystem,
                    () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar / 3.0,
                    () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar / 3.0,
                    () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar / 3.0,
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

    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(swerveSubsystem::zeroGyro)
    );

    // ----------------------------------------------------------------------------------------
    // ********************************* SECONDARY CONTROLLER *********************************
    // ----------------------------------------------------------------------------------------


    new JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
            new RunCommand(() -> this.mode = Constants.Mode.INTAKE)
    );

    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() -> this.mode = Constants.Mode.AMP)
    );
    new JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
            new RunCommand(() -> this.mode = Constants.Mode.SPEAKER)
    );

    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
            new RunCommand(() -> this.mode = Constants.Mode.PASSING)
    );
    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            new RunCommand(() -> this.mode = Constants.Mode.OFF)
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


  /**
   * Runs on enabling of the robot
   */
  public void enableInit() {
    aimSubsystem.resetEncoder();
    aimSubsystem.resetPID();
  }

  // I promise chatgpt didn't write this javadoc
  /**
   * Schedules commands based on the current operational mode.
   * <p>Possible modes include:
   * <ul>
   *   <li><b>Intake Mode:</b>
   *     <ul>
   *       <li>Ensure the shooter is fully lowered.</li>
   *       <li>Shooter wheels are not spinning.</li>
   *       <li>Run intake at designated speed.</li>
   *       <li>Activate automatic note rotation.</li>
   *       <li>Upon line break:
   *         <ul>
   *           <li>Stop intake wheels.</li>
   *           <li>Switch robot mode to off.</li>
   *         </ul>
   *       </li>
   *     </ul>
   *   </li>
   *   <li><b>Speaker Mode:</b>
   *     <p>Initiates when within the designated side of the field. If odometry confirms this zone, the following actions commence:
   *       <ul>
   *         <li>Run shooter auto rotation to optimize shot angle.</li>
   *         <li>Automatic rotation to speaker (can be overridden by the driver).</li>
   *         <li>Start spinning up to shooting speed.</li>
   *       </ul>
   *     </p>
   *   </li>
   *   <li><b>Amp Mode:</b>
   *     <p>Similar to Speaker Mode, but with different actions upon confirming zone:
   *       <ul>
   *         <li>Adjust shooter to approximately 90 degrees.</li>
   *         <li>Spin up for shooting the amp.</li>
   *         <li>Automatic rotation to align shooter with the amp.</li>
   *       </ul>
   *     </p>
   *   </li>
   *   <li><b>Passing Mode:</b>
   *     <p>Similar to Speaker Mode, but targets the corner of the field without waiting for a specific zone.</p>
   *   </li>
   *   <li><b>Off Mode:</b>
   *     <p>Only allows driving; intake is off, shooter is lowered, and no vision processing is active.</p>
   *   </li>
   * </ul>
   * </p>
   */
  public void modesCommandScheduler() {
    this.NTModeInfo.setString("Current mode", this.mode.toString());

    CommandScheduler CSInstance = CommandScheduler.getInstance();
    switch (this.mode) {
      case INTAKE -> CSInstance.schedule(
              new ParallelCommandGroup(
                      new RunCommand(aimSubsystem::reset, aimSubsystem),
                      new IntakeCommand(intakeSubsystem, false),
                      new RotateTo(swerveSubsystem, primaryController, Target.NOTE).finallyDo(() -> this.mode = Constants.Mode.OFF)
              )
      );
      case SPEAKER, AMP -> {
        Target target = this.mode == Constants.Mode.SPEAKER ? Target.SPEAKER : Target.AMP;
        if (swerveSubsystem.isCloseToUs()) {
          CSInstance.schedule(
                  new ParallelCommandGroup(
                          new SpinUpCommand(shooterSubsystem, target),
                          new AimCommand(aimSubsystem, swerveSubsystem, primaryController, target),
                          new RotateTo(swerveSubsystem, primaryController, target)
                  )
          );
        }
      }
      case PASSING -> {
        CSInstance.schedule(
                new ParallelCommandGroup(
                        new SpinUpCommand(shooterSubsystem, Target.LOW_PASS /* Dont know if low or high rn*/),
                        new AimCommand(aimSubsystem, swerveSubsystem, primaryController, Target.LOW_PASS),
                        new RotateTo(swerveSubsystem, primaryController, Target.LOW_PASS)
                )
        );
      }
      case OFF -> {
        System.out.println("Nothing fancy");
      }
    }
  }
}
