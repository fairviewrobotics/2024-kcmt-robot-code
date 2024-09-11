package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.utils.CANUtils;

import java.util.function.Function;

public class IntakeSubsystem extends SubsystemBase {
    // Motors
    private final CANSparkMax topMotor = CANUtils.configure(new CANSparkMax(IntakeConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));
    private final CANSparkMax bottomMotor = CANUtils.configure(new CANSparkMax(IntakeConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

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
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        topMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        topMotor.setInverted(false);

        bottomMotor.follow(topMotor, true);
    }


    /**
     * Set the speed of the intake motor
     * @param speed The target speed in %
     */
    public void setSpeed(double speed) {
        topMotor.set(speed);
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

}
