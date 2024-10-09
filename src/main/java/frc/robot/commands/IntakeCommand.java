package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utils.ConfigManager;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    private final LEDSubsystem ledSubsystem;

    private boolean continuous;

    private boolean done = false;

    private double rotationsUntilStop = 2.5;

    private String ledState = "off";

    private double startRotations;



    /**
     * Command to intake a note
     * @param intakeSubsystem The instance of {@link IntakeSubsystem}
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, boolean continuous) {
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.continuous = continuous;

        addRequirements(intakeSubsystem, ledSubsystem);
    }

    @Override
    public void execute() { // TODO: WTH is all this
        ConfigManager configManager = ConfigManager.getInstance();
        if (!this.intakeSubsystem.getFrontLinebreak() || continuous) {
            intakeSubsystem.setSpeed(0.3);
            startRotations = intakeSubsystem.getTopMotorRotations();
        } else {
            if (intakeSubsystem.getTopMotorRotations() - startRotations > rotationsUntilStop) {
                intakeSubsystem.setSpeed(0);
                this.done = true;
            } else {
                intakeSubsystem.setSpeed(0.3);
                ledSubsystem.setAnimation(LEDSubsystem.AnimationTypes.GreenStrobe);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0.0);
        ledSubsystem.setAnimation(LEDSubsystem.AnimationTypes.Off);
        if (intakeSubsystem.getFrontLinebreak()) ledSubsystem.setRGB(0, 255, 0);
    }

    @Override
    public void initialize() {
        this.done = false;
    }

    @Override
    public boolean isFinished() {
        return this.done;
    }
}
