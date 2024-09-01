package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {
    public static final double MAX_RPM = 6500;

    public static final int TOP_MOTOR_ID = 0;

    public static final int BOTTOM_MOTOR_ID = 0;

    public static final double SHOOTER_P = 5;

    public static final double SHOOTER_I = 0;

    public static final double SHOOTER_D = 0;

    public static final double SHOOTER_KS = 5;

    public static final double SHOOTER_KV = 0;

    public static final double SHOOTER_KA = 0;

    public static final Pose2d SPEAKER_POSE_BLUE = new Pose2d(16.5, 5.5, Rotation2d.fromRadians(0.0));

    public static final Pose2d SPEAKER_POSE_RED = new Pose2d(0.0, 5.5, Rotation2d.fromRadians(0.0));
}
