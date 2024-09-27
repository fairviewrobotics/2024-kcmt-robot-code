package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AimConstants;
import frc.robot.utils.CANUtils;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTableUtils;
import frc.robot.utils.ShooterUtils;

public class AimSubsystem extends SubsystemBase {
    // Motors
    private final CANSparkFlex leftMotor = CANUtils.configure(new CANSparkFlex(AimConstants.LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));
    private final CANSparkFlex rightMotor = CANUtils.configure(new CANSparkFlex(AimConstants.RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless));

    private final NetworkTableUtils NTDebug = new NetworkTableUtils("Debug");

    // PID controller
    private final ProfiledPIDController aimPID = new ProfiledPIDController(
            AimConstants.AIM_P,
            AimConstants.AIM_I,
            AimConstants.AIM_D,
            AimConstants.CONSTRAINTS
    );

    // Feedforward controller
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
        rightMotor.setInverted(false);
        leftMotor.setInverted(true);


        aimPID.setTolerance(AimConstants.AIM_TOLERANCE);
    }

    public void runLeft(double v) {
        leftMotor.setVoltage(v);
    }

    public void runRight(double v) {
        rightMotor.setVoltage(v);
    }

    /**
     * Set the angle of the shooter
     * @param angle Set the target angle for the shooter in radians
     */
    public void setAngle(double angle) {
        angle = MathUtils.inRange(angle, AimConstants.ANGLE_MIN, AimConstants.ANGLE_MAX);
        double pidValue = aimPID.calculate(ShooterUtils.encoderToRad(leftMotor.getEncoder().getPosition()), angle);
        double ffValue = aimFF.calculate(angle, 0);

        rightMotor.setVoltage(pidValue + ffValue);
        leftMotor.setVoltage(pidValue + ffValue);


        NTDebug.setDouble("Setpoint AIM", angle);
        NTDebug.setDouble("PID error", aimPID.getPositionError());
    }

    /**
     * Check if the shooter is at the target angle
     * @return If the shooter is at the target angle
     */
    public boolean atTargetAngle() {
        return this.aimPID.atSetpoint();
    }

    /**
     * Reset the shooter to the default angle
     */
    public void reset() {
        double pidValue = aimPID.calculate(ShooterUtils.encoderToRad(leftMotor.getEncoder().getPosition()), AimConstants.DEFAULT_ANGLE);
        double ffValue = aimFF.calculate(AimConstants.DEFAULT_ANGLE, 0);

        rightMotor.setVoltage(pidValue + ffValue);
        leftMotor.setVoltage(pidValue + ffValue);
    }

    @Override
    public void periodic() {
        NTDebug.setDouble("Pos deg", ShooterUtils.encoderToRad(leftMotor.getEncoder().getPosition()));
        ;
    }

    /**
     * Reset the PID controller for the shooter movement
     * This should be called everytime a command that uses the PID controller is started
     */
    public void resetPID() {
        aimPID.reset(leftMotor.getEncoder().getPosition());
    }

    public void resetEncoder() {
        this.leftMotor.getEncoder().setPosition(0);
    }
}
