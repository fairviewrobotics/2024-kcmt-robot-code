package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.utils.CANUtils;

public class IntakeSubsystem extends SubsystemBase {
    // Motors
    private final CANSparkFlex topMotor = CANUtils.configure(new CANSparkFlex(IntakeConstants.LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));
    private final CANSparkFlex bottomMotor = CANUtils.configure(new CANSparkFlex(IntakeConstants.RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final DoubleEntry intakeSpeedTop = NetworkTableInstance.getDefault()
            .getTable("Intake").getDoubleTopic("Top").getEntry(0.0);

    private final DoubleEntry intakeSpeedBottom = NetworkTableInstance.getDefault()
            .getTable("Intake").getDoubleTopic("Bottom").getEntry(0.0);

    private final DoubleEntry intakeCycleTop = NetworkTableInstance.getDefault()
            .getTable("Intake").getDoubleTopic("TopC").getEntry(0.0);

    private final DoubleEntry intakeCycleBottom = NetworkTableInstance.getDefault()
            .getTable("Intake").getDoubleTopic("BottomC").getEntry(0.0);

    // Linebreaks
    private final DigitalInput frontLinebreak = new DigitalInput(IntakeConstants.FRONT_LINEBREAK);
    private final DigitalInput backLinebreak = new DigitalInput(IntakeConstants.BACK_LINEBREAK);

    // States
    private boolean preBackState;
    private boolean preFrontState;

    /**
     * Subsystem for controlling the intake
     */
    public IntakeSubsystem() {
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        topMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        topMotor.setInverted(false);
        bottomMotor.setInverted(true);

    }


    /**
     * Set the speed of the intake motor
     * @param speed The target speed in %
     */
    public void setSpeed(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
    }

    /**
     * Check if the front linebreak is activated
     * @return If the front linebreak is activated
     */
    public boolean getFrontLinebreak() {
        return !frontLinebreak.get();
    }

    /**
     * Check if the back linebreak is activated
     * @return If the back linebreak is activated
     */
    public boolean getBackLinebreak() {
        return !backLinebreak.get();
    }

    /**
     * Runs something every time the front limelights state changes
     * @param r A {@link Runnable} to run when the state changes
     */
    public void frontChangeListener(Runnable r) {
        if (preFrontState != getFrontLinebreak()) {
            preFrontState = !preFrontState;
            r.run();
        }
    }

    @Override
    public void periodic() {
        intakeSpeedBottom.set(bottomMotor.getEncoder().getVelocity());
        intakeSpeedTop.set(topMotor.getEncoder().getVelocity());

        intakeCycleTop.set(topMotor.getAppliedOutput());
        intakeCycleBottom.set(bottomMotor.getAppliedOutput());
    }

}
