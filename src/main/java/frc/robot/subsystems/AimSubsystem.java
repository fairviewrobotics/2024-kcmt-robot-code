package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AimConstants;
import frc.robot.utils.CANUtils;
import frc.robot.utils.MathUtils;

public class AimSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = CANUtils.configure(new CANSparkMax(AimConstants.LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final CANSparkMax rightMotor = CANUtils.configure(new CANSparkMax(AimConstants.RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final SparkAbsoluteEncoder aimEncoder = leftMotor.getAbsoluteEncoder();

    private final ProfiledPIDController aimPID = new ProfiledPIDController(
            AimConstants.AIM_P,
            AimConstants.AIM_I,
            AimConstants.AIM_D,
            AimConstants.CONSTRAINTS
    );

    private final ArmFeedforward aimFF = new ArmFeedforward(
            AimConstants.AIM_KS,
            AimConstants.AIM_KG,
            AimConstants.AIM_KV,
            AimConstants.AIM_KA
    );

    /**
     * Subsystem for aiming the shooter
     */
    public AimSubsystem() {
        // TODO: reverse the motors if needed
        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        aimPID.setTolerance(AimConstants.AIM_TOLERANCE);
    }

    /**
     * Set the angle of the shooter
     * @param angle Set the target angle for the shooter in radians
     */
    public void setAngle(double angle) {
        angle = MathUtils.inRange(angle, AimConstants.ANGLE_MIN, AimConstants.ANGLE_MAX);
        double pidValue = aimPID.calculate(aimEncoder.getPosition(), angle);
        double ffValue = aimFF.calculate(angle, 0);

        leftMotor.setVoltage(pidValue + ffValue);
        rightMotor.setVoltage(pidValue + ffValue);
    }

    /**
     * Reset the shooter to the default angle
     */
    public void reset() {
        double pidValue = aimPID.calculate(aimEncoder.getPosition(), AimConstants.DEFAULT_ANGLE);
        double ffValue = aimFF.calculate(AimConstants.DEFAULT_ANGLE, 0);

        leftMotor.setVoltage(pidValue + ffValue);
        rightMotor.setVoltage(pidValue + ffValue);
    }

    /**
     * Reset the PID controller for the shooter movement
     * This should be called everytime a command that uses the PID controller is started
     */
    public void resetPID() {
        aimPID.reset(aimEncoder.getPosition());
    }
}
