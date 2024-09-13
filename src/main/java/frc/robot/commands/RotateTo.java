package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.Target;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.VisionUtils;

public class RotateTo extends Command {
    private final SwerveSubsystem swerveSubsystem;

    private final XboxController controller;

    private final Target target;

    private final ProfiledPIDController rotatePID = new ProfiledPIDController(
            VisionConstants.ROTATE_TO_P,
            VisionConstants.ROTATE_TO_I,
            VisionConstants.ROTATE_TO_D,
            VisionConstants.ROTATE_TO_CONSTRAINTS
    );

    private final ProfiledPIDController movementPID = new ProfiledPIDController(
            VisionConstants.MOVEMENT_P,
            VisionConstants.MOVEMENT_I,
            VisionConstants.MOVEMENT_D,
            VisionConstants.MOVEMENT_CONSTRAINTS
    );

    private double seeNote;

    private Pose2d rotateToPose;

    public RotateTo(SwerveSubsystem swerveSubsystem, XboxController controller, Target target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;
        this.controller = controller;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        switch (this.target) {
            case NOTE -> {
                rotatePID.reset(swerveSubsystem.getPose().getRotation().getRadians());
                rotatePID.setTolerance(0.2);
                rotatePID.enableContinuousInput(-Math.PI*2, 0);

                seeNote = VisionUtils.getNoteTV();

                double cameraDist = VisionConstants.NOTE_WIDTH_M / Math.tan(Math.toRadians(
                        VisionConstants.LIMELIGHT_FOV * (VisionUtils.getThor()/VisionConstants.LIMELIGHT_MAX_WIDTH_PXLS)));

                double dist = Math.sqrt(Math.pow(cameraDist, 2) - Math.pow(VisionConstants.CAM_HEIGHT, 2));

                Pose2d robotPose = swerveSubsystem.getPose();

                this.rotateToPose = new Pose2d(robotPose.getX() + Math.cos(robotPose.getRotation().getRadians() - Math.toRadians(VisionUtils.getNoteTX())) * dist,  robotPose.getY() + Math.sin(robotPose.getRotation().getRadians() - Math.toRadians(VisionUtils.getNoteTX())) * dist, new Rotation2d());
            }
            case SPEAKER -> {
                rotatePID.reset(swerveSubsystem.getPose().getRotation().getRadians());
                rotatePID.setTolerance(0.05);
                rotatePID.enableContinuousInput(-Math.PI, Math.PI);

                assert DriverStation.getAlliance().isPresent();

                this.rotateToPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.SPEAKER_POSE_BLUE.toPose2d() : ShooterConstants.SPEAKER_POSE_RED.toPose2d();
            }
            case HIGH_PASS, LOW_PASS -> {
                rotatePID.reset(swerveSubsystem.getPose().getRotation().getRadians());
                rotatePID.setTolerance(0.05);
                rotatePID.enableContinuousInput(-Math.PI, Math.PI);

                assert DriverStation.getAlliance().isPresent();

                this.rotateToPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.PASS_POSE_BLUE : ShooterConstants.PASS_POSE_RED;
            }
            case AMP, AMP_SHOOT -> {
                rotatePID.reset(swerveSubsystem.getPose().getRotation().getRadians());
                rotatePID.setTolerance(0.05);
                rotatePID.enableContinuousInput(-Math.PI, Math.PI);

                assert DriverStation.getAlliance().isPresent();

                this.rotateToPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.AMP_POSE_BLUE : ShooterConstants.AMP_POSE_RED;
            }
        }
    }

    @Override
    public void execute() {

        double angle = this.rotatePID.calculate(swerveSubsystem.getPose().getRotation().getRadians(),
                SwerveUtils.rotateToPose(swerveSubsystem.getPose(), this.rotateToPose));

        double movement = this.movementPID.calculate(swerveSubsystem.getPose().getX(),
                this.rotateToPose.getX());

        if (this.target == Target.AMP || this.target == Target.AMP_SHOOT) {
            assert DriverStation.getAlliance().isPresent();

            angle = this.rotatePID.calculate(swerveSubsystem.getPose().getRotation().getRadians(),
                    (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? -Math.PI / 2 : Math.PI / 2);
        }

        if ((this.target == Target.NOTE && seeNote == 0.0) || Math.abs(controller.getRightX()) > 0.05) {
            swerveSubsystem.drive(
                    controller.getLeftY() * DrivetrainConstants.drivingSpeedScalar,
                    controller.getLeftX() * DrivetrainConstants.drivingSpeedScalar,
                    controller.getRightX() * DrivetrainConstants.rotationSpeedScalar,
                    true,
                    true
            );
        } else if (this.target == Target.AMP || this.target == Target.AMP_SHOOT) {
            swerveSubsystem.drive(
                    movement,
                    controller.getLeftX() * DrivetrainConstants.drivingSpeedScalar,
                    angle,
                    true,
                    true
            );
        } else {
            swerveSubsystem.drive(
                    controller.getLeftY() * DrivetrainConstants.drivingSpeedScalar,
                    controller.getLeftX() * DrivetrainConstants.drivingSpeedScalar,
                    angle,
                    true,
                    true
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
