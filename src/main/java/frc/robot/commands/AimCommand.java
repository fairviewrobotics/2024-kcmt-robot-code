package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Target;
import frc.robot.constants.AimConstants;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimCommand extends Command {
    private final AimSubsystem aimSubsystem;

    private final SwerveSubsystem swerveSubsystem;

    private final Target target;

    /**
     * Command to aim the shooter
     * @param aimSubsystem Instance of {@link AimSubsystem}
     * @param swerveSubsystem Instance of {@link SwerveSubsystem}
     * @param target The {@link Target} to aim the shooter at
     */
    public AimCommand(AimSubsystem aimSubsystem, SwerveSubsystem swerveSubsystem, Target target) {
        this.aimSubsystem = aimSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;

        addRequirements(aimSubsystem); // Might need to add swerve subsystem, though lots of commands that need the robot pose run at the same time
                                       // We need a way to access swerveSubsystem.getPose() from anywhere without blocking other commands from it
    }

    @Override
    public void initialize() {
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
                return Math.toRadians(90); // Idk what this should be
            }
            case SPEAKER -> {
                // TODO: Some math
                return 0.0;
            }
            case null, default -> {
                return AimConstants.DEFAULT_ANGLE;
            }
        }
    }
}
