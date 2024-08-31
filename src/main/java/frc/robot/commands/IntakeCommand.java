package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Command to intake a note
     * @param subsystem The instance of {@link IntakeSubsystem}
     */
    public IntakeCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (!this.intakeSubsystem.getFrontLinebreak()) {
            intakeSubsystem.setSpeed(0.6, 0.6);
        } else {
            intakeSubsystem.setSpeed(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0, 0);
    }
}
