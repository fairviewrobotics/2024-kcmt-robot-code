package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class ShooterConstants {
    public static final double MAX_MOTOR_RPM = 6500; //TODO: tune

    public static final double MOTOR_TO_WHEEL_RATIO = 60.0/36.0; //TODO: tune

    public static final double SPEAKER_RPM = 8500; //TODO: tune

    public static final double PASS_RPM = 4000; //TODO: tune

    public static final double AMP_RPM = 1700; //TODO: tune

    public static final int TOP_MOTOR_ID = 10;

    public static final int BOTTOM_MOTOR_ID = 9;

    public static final double SHOOTER_P = 0.02;

    public static final double SHOOTER_I = 0;

    public static final double SHOOTER_D = 0;

    public static final double SHOOTER_KS = 0.1;

    public static final double SHOOTER_KV = 0.01774;

    public static final double SHOOTER_KA = 0;

    public static final double SHOOTER_WHEEL_DIAMETER = 0.0444;
    // In meters
    public static final double SHOOTER_HEIGHT = 0.4;

    public static final double STAGE_HEIGHT = 2.3;

    public static final Pose3d SPEAKER_POSE_RED = new Pose3d(16.5, 5.5, 2.0, new Rotation3d(0.0, 0.0, 0.0));

    public static final Pose3d SPEAKER_POSE_BLUE = new Pose3d(0.0, 5.5, 2.0, new Rotation3d(0.0, 0.0, 0.0));

    public static final Pose2d PASS_POSE_RED = new Pose2d(16.5, 8.0, new Rotation2d(0.0)); //TODO: tune to actual values

    public static final Pose2d PASS_POSE_BLUE = new Pose2d(0.0, 8.0, new Rotation2d(0.0)); //TODO: tune to actual values

    public static final Pose2d AMP_POSE_RED = new Pose2d(16.5, 8.0, new Rotation2d(0.0)); //TODO: tune to actual values

    public static final Pose2d AMP_POSE_BLUE = new Pose2d(0.0, 8.0, new Rotation2d(0.0)); //TODO: tune to actual values

}
