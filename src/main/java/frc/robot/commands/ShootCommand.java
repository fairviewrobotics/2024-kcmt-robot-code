package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ShootCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    private final LEDSubsystem ledSubsystem;
    /**
     * Command to shoot a held note then reset the shooter aim
     * @param intakeSubsystem Instance of {@link IntakeSubsystem}
     */
    public ShootCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(intakeSubsystem, ledSubsystem);
    }





    @Override
    public void execute() {

        intakeSubsystem.setSpeed(0.75);
        if (!intakeSubsystem.getBackLinebreak()) {
            ledSubsystem.setAnimation(LEDSubsystem.AnimationTypes.RedStrobe);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0.0);
    }

}
