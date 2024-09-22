package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.CANUtils;
import frc.robot.utils.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    // Motors
    private final CANSparkMax topMotor = CANUtils.configure(new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));
    private final CANSparkMax bottomMotor = CANUtils.configure(new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

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
//        speed /= ShooterConstants.MOTOR_TO_WHEEL_RATIO;
//        topMotor.setVoltage(
//                shooterPID.calculate(
//                    MathUtils.rpmToRadians(topMotor.getEncoder().getVelocity()),
//                    MathUtils.rpmToRadians(speed)) + shooterFF.calculate(MathUtils.rpmToRadians(speed)
//                )
//        );
        topMotor.set(speed);
        bottomMotor.set(speed);
    }

    /**
     * Reset the shooter PID controller
     */
    public void resetPID() {
        shooterPID.reset();
    }

    @Override
    public void periodic() {
        System.out.println(bottomMotor.isFollower());
    }


}
