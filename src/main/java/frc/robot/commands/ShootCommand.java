package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;


    /**
     * Command to shoot a held note then reset the shooter aim
     * @param intakeSubsystem Instance of {@link IntakeSubsystem}
     */
    public ShootCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }





    @Override
    public void execute() {
        intakeSubsystem.setSpeed(0.3, 0.3);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0, 0);
    }

}
