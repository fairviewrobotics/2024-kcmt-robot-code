package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.Target;
import frc.robot.constants.AimConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.ShooterUtils;

public class AimCommand extends Command {
    private final AimSubsystem aimSubsystem;

    private final SwerveSubsystem swerveSubsystem;

    private final XboxController controller;

    private Pose3d speakerPose;

    private Pose2d passPose;

    private final Target target;

    /**
     * Command to aim the shooter
     * @param aimSubsystem Instance of {@link AimSubsystem}
     * @param swerveSubsystem Instance of {@link SwerveSubsystem}
     * @param controller The {@link XboxController} that will exectute amp shooting
     * @param target The {@link Target} to aim the shooter at
     */
    public AimCommand(AimSubsystem aimSubsystem, SwerveSubsystem swerveSubsystem, XboxController controller, Target target) {
        this.aimSubsystem = aimSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        this.target = target;

        assert DriverStation.getAlliance().isPresent();

        this.speakerPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.SPEAKER_POSE_BLUE : ShooterConstants.SPEAKER_POSE_RED;
        this.passPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.PASS_POSE_BLUE : ShooterConstants.PASS_POSE_RED;

        addRequirements(aimSubsystem); // Might need to add swerve subsystem, though lots of commands that need the robot pose run at the same time
                                       // We need a way to access swerveSubsystem.getPose() from anywhere without blocking other commands from it
    }

    @Override
    public void initialize() {
        aimSubsystem.resetPID();

        assert DriverStation.getAlliance().isPresent();

        this.speakerPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.SPEAKER_POSE_BLUE : ShooterConstants.SPEAKER_POSE_RED;
        this.passPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.PASS_POSE_BLUE : ShooterConstants.PASS_POSE_RED;
    }

    @Override
    public void execute() {
        aimSubsystem.setAngle(this.calculateAngle(
                swerveSubsystem.getPose(),
                target
        ));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    private double calculateAngle(Pose2d robotPose, Target target) {
        switch (target) {
            case AMP -> {
                return (controller.getLeftBumper()? Math.toRadians(90): Math.toRadians(75)); //TODO: to tune
            }
            case SPEAKER -> {
                return ShooterUtils.calculateShooterAngle(
                        robotPose,
                        speakerPose,
                        ShooterConstants.MAX_RPM,
                        ShooterConstants.SHOOTER_WHEEL_DIAMETER,
                        ShooterConstants.SHOOTER_HEIGHT
                );
            }
            case HIGH_PASS -> {
                return ShooterUtils.calculateShooterAngleForPass(
                        robotPose,
                        passPose,
                        ShooterConstants.PASS_RPM,
                        ShooterConstants.SHOOTER_WHEEL_DIAMETER,
                        ShooterConstants.SHOOTER_HEIGHT,
                        ShooterConstants.STAGE_HEIGHT
                );
            }
            case LOW_PASS -> {
                return Math.toRadians(20); //TODO: to tune
            }
            default -> {
                return AimConstants.DEFAULT_ANGLE;
            }
        }
    }
}
