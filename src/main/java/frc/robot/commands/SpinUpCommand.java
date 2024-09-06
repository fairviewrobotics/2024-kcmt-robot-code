package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.Target;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private final Target target;

    /**
     * Command to spin up the shooter
     * @param target Either the speaker or amp {@link Target}
     * @param shooterSubsystem Instance of {@link ShooterSubsystem}
     */
    public SpinUpCommand(Target target, ShooterSubsystem shooterSubsystem) {
        this.target = target;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        switch (this.target) {
            case AMP, AMP_SHOOT ->
                shooterSubsystem.setShooterSpeed(1000, 1000);
            case SPEAKER, LOW_PASS ->
                shooterSubsystem.setShooterSpeed(6500, 6500);
            case HIGH_PASS ->
                shooterSubsystem.setShooterSpeed(ShooterConstants.PASS_RPM, ShooterConstants.PASS_RPM);
        }
    }
}
