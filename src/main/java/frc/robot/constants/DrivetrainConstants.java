package frc.robot.constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {

    public static final double maxSpeedMetersPerSecond = 6;
    public static final double maxAngularSpeed = 2 * Math.PI;

    public static final double directionSlewRate = 18.0; // rads/sec - turning was 4.0
    public static final double magnitudeSlewRate = 22.0; // percent/second (1 = 100%) - forward/backward/traverse - was 20.0
    public static final double rotationalSlewRate = 30.0; // percent/second (1 = 100%) - rotation was 50.0

    public static final double drivingSpeedScalar = -4.0; //make positive so gyroreset with intake forward
    public static final double rotationSpeedScalar = 3 * Math.PI;//make positive so gyroreset with intake forward

    public static final double trackWidth = Units.inchesToMeters(20);
    public static final double wheelBase = Units.inchesToMeters(16);

    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2)
    );

    public static final double frontLeftChassisAngularOffset = 3.850;
    public static final double frontRightChassisAngularOffset = 2.023 + Math.PI;
    public static final double rearLeftChassisAngularOffset = 3.231 - Math.PI;
    public static final double rearRightChassisAngularOffset = 0.943 + Math.PI;

    public static final int frontLeftDrivingPort = 3;
    public static final int rearLeftDrivingPort = 5;
    public static final int frontRightDrivingPort = 1;
    public static final int rearRightDrivingPort = 7;

    public static final int frontLeftTurningPort = 4;
    public static final int rearLeftTurningPort = 6;
    public static final int frontRightTurningPort = 2;
    public static final int rearRightTurningPort = 8;


    public static final boolean gyroReversed = false;
    public static final boolean turningEncoderReversed = true;

    public static final int drivingMotorPinionTeeth = 14;

    public static final int spurGearTeeth = 21;


    //Free Speed RPM: 6784
    public static final double freeSpeedRpm = 6784.0;
    public static final double drivingMotorFreeSpeedRps = freeSpeedRpm / 60.0;
    public static final double wheelDiameterMeters = 0.0762;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double drivingMotorReduction = (45.0 * spurGearTeeth) / (drivingMotorPinionTeeth * 15);
    public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters) / drivingMotorReduction;

    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / drivingMotorReduction;
    public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI) / drivingMotorReduction) / 60.0;
    public static final double turningEncoderPositionFactor = 2 * Math.PI;
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
    public static final double turningEncoderPositionPIDMinInput = 0.0;
    public static final double turningEncoderPositionPIDMaxInput = turningEncoderPositionFactor;

    public static final double drivingP = 0.06;
    public static final double drivingI = 0.0;
    public static final double drivingD = 0.0;
    public static final double drivingFF = 1.0 / driveWheelFreeSpeedRps;
    public static final double drivingMinOutput = -1.0;
    public static final double drivingMaxOutput = 1.0;

    public static final double turningP = 0.6;
    public static final double turningI = 0.0;
    public static final double turningD = 0.0;
    public static final double turningFF = 0.0;
    public static final double turningMinOutput = -1.0;
    public static final double turningMaxOutput = 1.0;

    public static final CANSparkFlex.IdleMode drivingMotorIdleMode = CANSparkFlex.IdleMode.kCoast;
    public static final CANSparkMax.IdleMode turningMotorIdleMode = CANSparkMax.IdleMode.kBrake;

    public static final int drivingMotorCurrentLimit = 40;
    public static final int turningMotorCurrentLimit = 20;
}