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
    private final CANSparkMax topMotor = CANUtils.configure(new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final CANSparkMax bottomMotor = CANUtils.configure(new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final PIDController shooterPID = new PIDController(
            ShooterConstants.SHOOTER_P,
            ShooterConstants.SHOOTER_I,
            ShooterConstants.SHOOTER_D
    );

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
            ShooterConstants.SHOOTER_KS,
            ShooterConstants.SHOOTER_KV,
            ShooterConstants.SHOOTER_KA
    );

    /**
     * The subsystem for controlling all shooter things
     */
    public ShooterSubsystem() {
        topMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }

    /**
     * Set the target speed of the top motor
     * @param speed The target RPM for the motor
     */
    public void setTopSpeed(double speed) {
        topMotor.setVoltage(shooterPID.calculate(MathUtils.rpmToRadians(topMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                shooterFF.calculate(MathUtils.rpmToRadians(speed)));
    }

    /**
     * Set the target speed of the bottom motor
     * @param speed The target RPM for the motor
     */
    public void setBottomMotor(double speed) {
        bottomMotor.setVoltage(shooterPID.calculate(MathUtils.rpmToRadians(bottomMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                shooterFF.calculate(MathUtils.rpmToRadians(speed)));
    }

    /**
     * Set the speed of the top and bottom motors
     * @param top The target RPM for the top motor
     * @param bottom The target RPM for the bottom motor
     */
    public void setShooterSpeed(double top, double bottom) {
        this.setTopSpeed(top);
        this.setBottomMotor(bottom);
    }


}
