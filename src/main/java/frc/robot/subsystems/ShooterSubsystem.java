package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.CANUtils;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTableUtils;
import org.opencv.core.Mat;

public class ShooterSubsystem extends SubsystemBase {
    // Motors
    private final CANSparkFlex topMotor = CANUtils.configure(new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));
    private final CANSparkFlex bottomMotor = CANUtils.configure(new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final NetworkTableUtils NTDebug = new NetworkTableUtils("Debug");

    // PID Controller
    private final PIDController shooterPID = new PIDController(
            ShooterConstants.SHOOTER_P,
            ShooterConstants.SHOOTER_I,
            ShooterConstants.SHOOTER_D
    );

    // Feedforward controller
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
            ShooterConstants.SHOOTER_KS,
            ShooterConstants.SHOOTER_KV,
            ShooterConstants.SHOOTER_KA
    );

    /**
     * The subsystem for controlling shooter wheels
     */
    public ShooterSubsystem() {
        topMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        topMotor.setInverted(false);
        bottomMotor.setInverted(true);

    }

    /**
     * Set the target speed of the shooter
     * @param speed The target RPM for the shooter
     */
    public void setShooterSpeed(double speed) {
        speed /= ShooterConstants.MOTOR_TO_WHEEL_RATIO;
        topMotor.setVoltage(
                shooterPID.calculate(MathUtils.rpmToRadians(topMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                        shooterFF.calculate(MathUtils.rpmToRadians(speed))
        );
        bottomMotor.setVoltage(
                shooterPID.calculate(MathUtils.rpmToRadians(bottomMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                        shooterFF.calculate(MathUtils.rpmToRadians(speed))
        );

        NTDebug.setDouble("Target", speed);
        NTDebug.setDouble("Shooter PID + FF", shooterPID.calculate(MathUtils.rpmToRadians(bottomMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                shooterFF.calculate(MathUtils.rpmToRadians(speed)));

        NTDebug.setDouble("Shooter RPM Current", topMotor.getEncoder().getVelocity());

    }

    public void runVolts(double v) {
        topMotor.setVoltage(v);
        bottomMotor.setVoltage(v);
    }

    /**
     * Reset the shooter PID controller
     */
    public void resetPID() {
        shooterPID.reset();
    }

    @Override
    public void periodic() {
        NTDebug.setDouble("Current Top", topMotor.getEncoder().getVelocity());
        NTDebug.setDouble("Current Bottom", bottomMotor.getEncoder().getVelocity());
    }


}
