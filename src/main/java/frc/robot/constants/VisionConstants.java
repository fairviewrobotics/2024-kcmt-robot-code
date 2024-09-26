package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class VisionConstants {
    public static final double ROTATE_TO_P = 5;
    public static final double ROTATE_TO_I = 0;
    public static final double ROTATE_TO_D = 0;
    public static final TrapezoidProfile.Constraints ROTATE_TO_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI);


    public static final double CAM_HEIGHT_M = 0.2794;

    public static final double NOTE_WIDTH_M = 0.35;

    public static final int NOTE_CAM_FOV = 100;

    public static final int NOTE_CAM_MAX_WIDTH_PXLS = 1280;

    public static final double NOTE_CAM_OFFSET = Math.toRadians(32);

    public static final double MOVEMENT_P = 0.0; //TODO: tune
    public static final double MOVEMENT_I = 0.0; //TODO: tune
    public static final double MOVEMENT_D = 0.0; //TODO: tune
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(4.0, 2.0); //TODO: tune
}
