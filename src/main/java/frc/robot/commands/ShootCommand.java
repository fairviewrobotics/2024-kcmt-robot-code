package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    private final AimSubsystem aimSubsystem;

    private int llCount = 0;

    private boolean finished;

    /**
     * Command to shoot a held note then reset the shooter aim
     * @param intakeSubsystem Instance of {@link IntakeSubsystem}
     * @param aimSubsystem Instance of {@link AimSubsystem}
     */
    public ShootCommand(IntakeSubsystem intakeSubsystem, AimSubsystem aimSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.aimSubsystem = aimSubsystem;

        addRequirements(aimSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.frontChangeListener(() -> llCount++);
        intakeSubsystem.setSpeed(0.3, 0.3);

        if (llCount >= 4) { // Starts at 0, adds +1 when the font of the note hits it, +1 when front of note leaves,
                            // +1 when back of note hits, +1 when back of note leaves
                            // (Might need to be different if the note is activating the front limelight when it's in the holding position idk)
            intakeSubsystem.setSpeed(0, 0);
            aimSubsystem.reset();

            if (aimSubsystem.atTargetAngle()) {
                finished = true;
            }
        }
    }

    @Override
    public void initialize() {
        llCount = 0;
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
