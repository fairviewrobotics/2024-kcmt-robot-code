package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AimConstants {
    public static final int LEFT_MOTOR_ID = 11;

    public static final int RIGHT_MOTOR_ID = 12;

    public static final double AIM_P = 20;
    public static final double AIM_I = 0.2;
    public static final double AIM_D = 0;
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI*2, Math.PI);

    public static final double AIM_KS = 0.0; //Value of static force; depends on mechanism
    public static final double AIM_KG = 0.3; //Increase until position holds
    public static final double AIM_KV = 0.0; //Increase until arm sort of tracks a setpoint
    public static final double AIM_KA = 0.0; //Do not change

    public static final double MIN_ENCODER = 0; // 5 deg

    public static final double MAX_ENCODER = 23.170; // 90 def

    public static final double DEFAULT_ANGLE = Math.toRadians(5);

    public static final double ANGLE_MAX = Math.PI/2;

    public static final double ANGLE_MIN = Math.toRadians(5);

    public static final double AIM_TOLERANCE = Math.toRadians(1);
    

}
