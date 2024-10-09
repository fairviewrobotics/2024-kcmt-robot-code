package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.controllers.SwerveModuleControlller;
import frc.robot.utils.NetworkTableUtils;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.VisionUtils;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

public class SwerveSubsystem extends SubsystemBase {
    // Defining Motors
    private final SwerveModuleControlller frontLeft = new SwerveModuleControlller(
            DrivetrainConstants.frontLeftDrivingPort,
            DrivetrainConstants.frontLeftTurningPort,
            DrivetrainConstants.frontLeftChassisAngularOffset
    );

    private final SwerveModuleControlller frontRight = new SwerveModuleControlller(
            DrivetrainConstants.frontRightDrivingPort,
            DrivetrainConstants.frontRightTurningPort,
            DrivetrainConstants.frontRightChassisAngularOffset
    );

    private final SwerveModuleControlller rearLeft = new SwerveModuleControlller(
            DrivetrainConstants.rearLeftDrivingPort,
            DrivetrainConstants.rearLeftTurningPort,
            DrivetrainConstants.rearLeftChassisAngularOffset
    );

    private final SwerveModuleControlller rearRight = new SwerveModuleControlller(
            DrivetrainConstants.rearRightDrivingPort,
            DrivetrainConstants.rearRightTurningPort,
            DrivetrainConstants.rearRightChassisAngularOffset
    );

    // Gyro
    private final AHRS gyro = new AHRS();

    // Slew Rate Constants
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    // Slew Rate Limiters
    private final SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DrivetrainConstants.magnitudeSlewRate);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(DrivetrainConstants.rotationalSlewRate);

    // Slew Rate Time
    private double previousTime = WPIUtilJNI.now() * 1e-6;

    // Swerve Odometry
    /*
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            DrivetrainConstants.driveKinematics,
            Rotation2d.fromRadians(heading()),
            new SwerveModulePosition[]{
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition()
            }
    );
     */

    private double distanceToTag = 1.0;

    private final PhotonCamera cam = VisionUtils.getPhotonAprilCamera();

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d robotToCam = VisionUtils.getPhotonAprilRobotToCamera();
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.driveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition()
            },
            new Pose2d(),

            // How much we trust the wheel measurements
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),

            // How much we trust the vision measurements
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10))
    );

    // Network Tables Telemetry

    private final NetworkTableUtils NTUtils = new NetworkTableUtils("Debug");
    private final DoubleArrayEntry setpointsTelemetry = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleArrayTopic("Setpoints").getEntry(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    private final DoubleArrayEntry actualTelemetry = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleArrayTopic("Actual").getEntry(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    private final DoubleArrayEntry poseTelemetry = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleArrayTopic("Pose").getEntry(new double[]{poseEstimator.getEstimatedPosition().getTranslation().getX(),
                    poseEstimator.getEstimatedPosition().getTranslation().getY(),
                    poseEstimator.getEstimatedPosition().getRotation().getRadians()});

    private final DoubleEntry gyroHeading = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleTopic("GyroHeading").getEntry(heading());

    private final DoubleEntry frontrightpos = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleTopic("frpos").getEntry(frontRight.getPosition().angle.getRadians());

    private final DoubleEntry frontleftpos = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleTopic("flpos").getEntry(frontLeft.getPosition().angle.getRadians());

    private final DoubleEntry rearrightpos = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleTopic("rrpos").getEntry(rearRight.getPosition().angle.getRadians());

    private final DoubleEntry rearleftpos = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleTopic("rlpos").getEntry(rearLeft.getPosition().angle.getRadians());

    /**
     * This subsystem manages all the swerve drive logic and also gives data to odometry
     */
    public SwerveSubsystem() {
        // PathPlanner stuff
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.5, 0.0, 0.2), //1.8 // 2.7
                        new PIDConstants(3, 0.0, 0.2), //1.0 // 1.8
                        3, //swervesubsystem.setmodulestate
                        0.301625,//11.875 meters
                        new ReplanningConfig()
                ),
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this
        );


        gyro.setAngleAdjustment(0);
    }


    // Periodic
    @Override
    public void periodic() {
        NTUtils.setDouble("Gyro Angle", gyro.getAngle());

//        if (!DriverStation.isAutonomous())
//            gyro.setAngleAdjustment(180);
//        else
//            gyro.setAngleAdjustment(0);

        // Add wheel measurements to odometry
        poseEstimator.update(
                Rotation2d.fromRadians(heading()),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                }
        );

        Optional<EstimatedRobotPose> estimatedRobotPose = VisionUtils.getEstimatedGlobalPose(photonPoseEstimator, poseEstimator.getEstimatedPosition());

        // Add vision measurement to odometry
        Optional<Pose3d> visionMeasurement = VisionUtils.getBotPoseFieldSpace();

        if ((visionMeasurement.isPresent() && VisionUtils.getDistanceFromTagLimelight() < 3) || estimatedRobotPose.isPresent()) {
//            distanceToTag = VisionUtils.getDistanceFromTag();
//
//
//            double visionTrust = 0.075 * Math.pow(distanceToTag, 2.5);
//            double rotationVisionTrust = Math.pow(distanceToTag, 2.5) / 5;
//
//            NTUtils.setDouble("Tag_Dist", distanceToTag);
//            NTUtils.setDouble("Vision_Trust", visionTrust);
//            NTUtils.setDouble("Rotation_Vision_Trust", rotationVisionTrust);
//            if (distanceToTag < 3) {
//                poseEstimator.setVisionMeasurementStdDevs(
//                        VecBuilder.fill(
//                                visionTrust,
//                                visionTrust,
//                                rotationVisionTrust
//                        )
//
//                );
//                //System.out.println(visionTrust);
//            } else {
//
//
//                // If we're 3+ meters away, limelight is too unreliable. Don't trust it!
//                poseEstimator.setVisionMeasurementStdDevs(
//                        VecBuilder.fill(9999, 9999, 9999)
//                );
//            }

//             poseEstimator.addVisionMeasurement(
//                    visionMeasurement.toPose2d(),
//                    Timer.getFPGATimestamp() - (VisionUtils.getLatencyPipeline()/1000.0) - (VisionUtils.getLatencyCapture()/1000.0)
//            );
            visionMeasurement.ifPresent(robotPose -> {
                if (VisionUtils.getDistanceFromTagLimelight() > 3) return;
                poseEstimator.addVisionMeasurement(
                    robotPose.toPose2d(),
                    Timer.getFPGATimestamp() - (VisionUtils.getLatencyPipeline()/1000.0) - (VisionUtils.getLatencyCapture()/1000.0)
                );
            });

            estimatedRobotPose.ifPresent(robotPose -> {

                if (cam.getCameraTable().getEntry("targetPose").getDoubleArray(new double[]{})[0] - 0.4 > 4 ) return;

                poseEstimator.addVisionMeasurement(
                    robotPose.estimatedPose.toPose2d(),
                    robotPose.timestampSeconds
                );
            });

        }

        frontrightpos.set(frontRight.getPosition().angle.getRadians());
        frontleftpos.set(frontLeft.getPosition().angle.getRadians());
        rearrightpos.set(rearRight.getPosition().angle.getRadians());
        rearleftpos.set(rearLeft.getPosition().angle.getRadians());


        // Set Network Tables Telemetry
        actualTelemetry.set(new double[]{
                frontLeft.getPosition().angle.getRadians(), frontLeft.getState().speedMetersPerSecond,
                frontRight.getPosition().angle.getRadians(), frontRight.getState().speedMetersPerSecond,
                rearLeft.getPosition().angle.getRadians(), rearLeft.getState().speedMetersPerSecond,
                rearRight.getPosition().angle.getRadians(), rearRight.getState().speedMetersPerSecond});

        setpointsTelemetry.set(new double[]{
                frontLeft.getDesiredState().angle.getRadians(), frontLeft.getDesiredState().speedMetersPerSecond,
                frontRight.getDesiredState().angle.getRadians(), frontRight.getDesiredState().speedMetersPerSecond,
                rearLeft.getDesiredState().angle.getRadians(), rearLeft.getDesiredState().speedMetersPerSecond,
                rearRight.getDesiredState().angle.getRadians(), rearRight.getDesiredState().speedMetersPerSecond});

        poseTelemetry.set(new double[]{
                poseEstimator.getEstimatedPosition().getTranslation().getX(),
                poseEstimator.getEstimatedPosition().getTranslation().getY(),
                poseEstimator.getEstimatedPosition().getRotation().getRadians()
        });

        gyroHeading.set(heading());

    }

    // Define robot pose

    /**
     * Get robot's pose.
     * @return Returns the robots current position of the field as a {@link Pose2d}
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    /**
     * Get current heading of robot
     * @return Heading of robot in radians
     */
    public double heading() {
        return Units.degreesToRadians(-1 * ((gyro.getAngle() + 0.0) % 360.0));
    }

    /**
     * Get the pose estimator instance
     * @return Instance of the {@link SwerveDrivePoseEstimator}
     */
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return this.poseEstimator;
    }

    /**
     * Drive the robot with {@link ChassisSpeeds} (mainly used for path planner)
     * @param chassisSpeeds {@link ChassisSpeeds} object
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
            double forward = chassisSpeeds.vxMetersPerSecond;
            double sideways = chassisSpeeds.vyMetersPerSecond;
            double rotation = chassisSpeeds.omegaRadiansPerSecond;

            drive(forward, sideways, rotation, false, false);//ratelimit was true, to be tested

    }

    /**
     * Get the speed of the chassis relative to the robot
     * @return {@link ChassisSpeeds} of the current robots speed
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DrivetrainConstants.driveKinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState()
        );

    }

    /**
     * Gets the robots chassis speed relative to the field
     * @return Returns robo t speed as a {@link ChassisSpeeds} in meters/second
     */
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return new ChassisSpeeds(
                getRobotRelativeSpeeds().vxMetersPerSecond * getPose().getRotation().getCos()
                        - getRobotRelativeSpeeds().vyMetersPerSecond * getPose().getRotation().getSin(),
                getRobotRelativeSpeeds().vyMetersPerSecond * getPose().getRotation().getCos()
                        + getRobotRelativeSpeeds().vxMetersPerSecond * getPose().getRotation().getSin(),
                getRobotRelativeSpeeds().omegaRadiansPerSecond);
    }

    /**
     * Resets the robots position on the field
     * @param pose Reset robot's position.
     */

    private void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                Rotation2d.fromRadians(heading()),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                },
                pose
        );
    }

    // Drive function - slew rate limited to prevent shearing of wheels

    /**
     * Swerve drive function.
     * @param forwardMetersPerSecond The target forward m/s
     * @param sidewaysMetersPerSecond The target sideways m/s
     * @param radiansPerSecond The target Rad/s
     * @param fieldRelative If the robot is robot relative (forwards is front of robot) or field relative (forward is opposite side of field)
     * @param rateLimit If we should apply slew rates (should always be true unless you know what your doing)
     */

    public void drive(double forwardMetersPerSecond, double sidewaysMetersPerSecond, double radiansPerSecond, boolean fieldRelative, boolean rateLimit) {
        // forward is xspeed, sideways is yspeed
//        double xSpeedCommanded = 0.0;
//        double ySpeedCommanded = 0.0;
//
//        double[] areaPoints = {0.0, 0.0, 2.0, 2.0};
//        double poseX = getPose().getX();
//        double poseY = getPose().getY();
//
//        boolean aboveMinPoints = poseX >= areaPoints[0] && poseY >= areaPoints[1];
//        boolean belowMaxPoints = poseX <= areaPoints[2] && poseY <= areaPoints[3];
//
//        if (aboveMinPoints && belowMaxPoints) {
//            xSpeedCommanded /= 2;
//            ySpeedCommanded /= 2;
//        }
//        if (ShooterConstants.shooterPID.getSetpoint() != 0 && ShooterConstants.isActive) {
//            forwardMetersPerSecond = forwardMetersPerSecond;
//        }


        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {

            //Math that calculates important stuff about where the robot is heading
            double inputTranslationDirection = Math.atan2(sidewaysMetersPerSecond, forwardMetersPerSecond);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(forwardMetersPerSecond, 2.0) + Math.pow(sidewaysMetersPerSecond, 2.0));

            double directionSlewRate;
            if (currentTranslationMagnitude != 0.0) {
                directionSlewRate = Math.abs(DrivetrainConstants.directionSlewRate / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0; // super high number means slew is instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;

            double angleDifference = SwerveUtils.AngleDifference(inputTranslationDirection, currentTranslationDirection);
            if (angleDifference < 0.45 * Math.PI) {
                currentTranslationDirection = SwerveUtils.StepTowardsCircular(
                        currentTranslationDirection,
                        inputTranslationDirection,
                        directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            } else if (angleDifference > 0.85 * Math.PI) {
                if (currentTranslationMagnitude > 1e-4) { // small number avoids floating-point errors
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentTranslationDirection = SwerveUtils.WrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }
            } else {
                currentTranslationDirection = SwerveUtils.StepTowardsCircular(
                        currentTranslationDirection,
                        inputTranslationDirection,
                        directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            }

            previousTime = currentTime;

            xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(radiansPerSecond);

        } else {
            // If there's no rate limit, robot does the exact inputs given.
            xSpeedCommanded = forwardMetersPerSecond;
            ySpeedCommanded = sidewaysMetersPerSecond;
            currentRotation = radiansPerSecond;
        }


        double xSpeedDelivered = xSpeedCommanded;
        double ySpeedDelivered = ySpeedCommanded;
        double rotationDelivered = currentRotation;

        // Field relative is easier for drivers.
        SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            swerveModuleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeedDelivered,
                            ySpeedDelivered,
                            rotationDelivered,
                            Rotation2d.fromRadians(heading())
                    )
            );
        } else {
            swerveModuleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                    new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // Sets the wheels to an X configuration

    /**
     * Set wheels to an X configuration for docking procedure.
     */

    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
        frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    }

    // Sets the wheels to a zeroed configuration

    /**
     * Set wheels to a 0 configuration for calibration and testing.
     */

    public void setZero() {
        frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }


    // Resets Gyro

    /**
     * Reset the gyro
     */
    public void zeroGyro() {
        gyro.reset();
    }

    /**
     * Resets Gyro and odometry
     */
    public void zeroGyroAndOdometry() {
        gyro.reset();
        resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    }

    /**
     * Sets states of swerve modules
     * @param desiredStates target states for the swerve modules (requires a list of 4 {@link SwerveModuleState}s)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (desiredStates.length != 4) {
            System.out.printf("Incorrect length of desiredStates, got %d expected 4%n", desiredStates.length);
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets swerve encoders
     */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();
    }

    /**
     * Is the robot close enough to our side of the field
     * @return If we are close enough
     */
    public boolean isCloseToUs() {
        assert DriverStation.getAlliance().isPresent();
        double xThreshold = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red) ? DrivetrainConstants.X_POS_THRESH_RED : DrivetrainConstants.X_POS_THRESH_BLUE; // TODO: I have no idea what this should be

        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red) ? getPose().getX() >= xThreshold : getPose().getX() <= xThreshold; // TODO: Also not sure here
    }
}