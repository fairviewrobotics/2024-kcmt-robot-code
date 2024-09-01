package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Target;
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

    private final PIDController rotatePID = new PIDController(
            VisionConstants.ROTATE_TO_P,
            VisionConstants.ROTATE_TO_I,
            VisionConstants.ROTATE_TO_D
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
                rotatePID.reset();
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
                rotatePID.reset();
                rotatePID.setTolerance(0.05);
                rotatePID.enableContinuousInput(-Math.PI, Math.PI);

                assert DriverStation.getAlliance().isPresent();

                this.rotateToPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.SPEAKER_POSE_BLUE : ShooterConstants.SPEAKER_POSE_RED;
            }
            case AMP -> {}
        }
    }

    @Override
    public void execute() {
        double angle = this.rotatePID.calculate(swerveSubsystem.getPose().getRotation().getRadians(),
                SwerveUtils.rotateToPose(swerveSubsystem.getPose(), this.rotateToPose));

        if (this.target == Target.NOTE && seeNote == 0.0) {
            swerveSubsystem.drive(
                    controller.getLeftY() * DrivetrainConstants.drivingSpeedScalar,
                    controller.getLeftX() * DrivetrainConstants.drivingSpeedScalar,
                    controller.getRightX() * DrivetrainConstants.rotationSpeedScalar,
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
