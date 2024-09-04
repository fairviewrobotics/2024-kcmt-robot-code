package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AimConstants;
import frc.robot.utils.CANUtils;
import frc.robot.utils.MathUtils;

public class AimSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = CANUtils.configure(new CANSparkMax(AimConstants.LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final CANSparkMax rightMotor = CANUtils.configure(new CANSparkMax(AimConstants.RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final SparkAbsoluteEncoder aimEncoder = leftMotor.getAbsoluteEncoder();

    private final PIDController aimPID = new PIDController(
            AimConstants.AIM_P,
            AimConstants.AIM_I,
            AimConstants.AIM_D
    );
    /**
     * Subsystem for aiming the shooter
     */
    public AimSubsystem() {
        // Might need to reverse one of the motors idk
    }

    /**
     * Set the angle of the shooter
     * @param angle Set the target angle for the shooter in radians
     */
    public void setAngle(double angle) {
        angle = Math.clamp(angle, AimConstants.ANGLE_MIN, AimConstants.ANGLE_MAX);
        double pidValue = aimPID.calculate(aimEncoder.getPosition(), angle);

        leftMotor.set(pidValue);
        rightMotor.set(pidValue);
    }

    public void reset() {
        double pidValue = aimPID.calculate(aimEncoder.getPosition(), AimConstants.DEFAULT_ANGLE);
        leftMotor.set(pidValue);
        rightMotor.set(pidValue);
    }
}
