package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Target;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.VisionUtils;

public class RotateTo extends Command {
    private final SwerveSubsystem swerveSubsystem;

    private final Target target;

    private final PIDController rotatePID = new PIDController(
            VisionConstants.ROTATE_TO_P,
            VisionConstants.ROTATE_TO_I,
            VisionConstants.ROTATE_TO_D
    );

    private double seeNote;

    private Pose2d notePose;

    public RotateTo(SwerveSubsystem swerveSubsystem, Target target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;

        rotatePID.setTolerance(0.2);
        rotatePID.enableContinuousInput(-Math.PI*2, 0);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        switch (this.target) {
            case NOTE -> {
                rotatePID.reset();
                seeNote = VisionUtils.getNoteTV();

                double cameraDist = VisionConstants.NOTE_WIDTH_M / Math.tan(Math.toRadians(
                        VisionConstants.LIMELIGHT_FOV * (VisionUtils.getThor()/VisionConstants.LIMELIGHT_MAX_WIDTH_PXLS)));

                double dist = Math.sqrt(Math.pow(cameraDist, 2) - Math.pow(VisionConstants.CAM_HEIGHT, 2));

                Pose2d robotPose = swerveSubsystem.getPose();

                notePose = new Pose2d(robotPose.getX() + Math.cos(robotPose.getRotation().getRadians() - Math.toRadians(VisionUtils.getNoteTX())) * dist,  robotPose.getY() + Math.sin(robotPose.getRotation().getRadians() - Math.toRadians(VisionUtils.getNoteTX())) * dist, new Rotation2d());
            }
            case SPEAKER -> {}
            case AMP -> {}
        }
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
