package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    private boolean continuous;

    /**
     * Command to intake a note
     * @param subsystem The instance of {@link IntakeSubsystem}
     */
    public IntakeCommand(IntakeSubsystem subsystem, boolean continuous) {
        this.intakeSubsystem = subsystem;
        this.continuous = continuous;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
//        if (!this.intakeSubsystem.getFrontLinebreak() || continuous) {
            intakeSubsystem.setSpeed(0.4);
//        } else {
//            intakeSubsystem.setSpeed(0);
//        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0.0);
    }
}
