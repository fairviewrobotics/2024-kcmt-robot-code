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
    private final CANSparkMax topMotor = CANUtils.configure(new CANSparkMax(IntakeConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final CANSparkMax bottomMotor = CANUtils.configure(new CANSparkMax(IntakeConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final DigitalInput frontLinebreak = new DigitalInput(IntakeConstants.FRONT_LINEBREAK);

    private final DigitalInput backLinebreak = new DigitalInput(IntakeConstants.BACK_LINEBREAK);

    private boolean preBackState;

    private boolean preFrontState;

    /**
     * Subsystem for controlling the intake
     */
    public IntakeSubsystem() {
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        topMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    /**
     * Set the speed of the top intake motor in %
     * @param speed The target speed of the top motor in %
     */
    public void setTopSpeed(double speed) {
        topMotor.set(speed);
    }

    /**
     * Set the speed of the bottom intake motor in %
     * @param speed The target speed of the bottom motor in %
     */
    public void setBottomSpeed(double speed) {
        bottomMotor.set(speed);
    }

    /**
     * Set the speed of the intake motors
     * @param top The target left speed in %
     * @param bottom The target right speed in %
     */
    public void setSpeed(double top, double bottom) {
        this.setTopSpeed(top);
        this.setBottomSpeed(bottom);
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
