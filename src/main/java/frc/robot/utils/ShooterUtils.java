package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.ShooterConstants;

import java.util.function.Function;


/**
 * Utility class for shooter calculations
 */
public class ShooterUtils {

    /**
     * Calculate the angle of the shooter to hit the speaker
     * @param robotPose The current {@link Pose2d} of the robot
     * @param targetPose The {@link Pose3d} of the target
     * @param shooterRPM The RPM of the shooter
     * @param wheelDiameter The diameter of the shooter wheel
     * @param shooterHeight The height of the shooter
     * @return The angle of the shooter
     */
    public static double calculateShooterAngle(
            Pose2d robotPose, Pose3d targetPose,
            double shooterRPM, double wheelDiameter,
            double shooterHeight) {

        // Constants
        double g = 9.81;  // Gravity in m/s^2

        // Calculate initial velocity
        double v0 = (shooterRPM * Math.PI * wheelDiameter) / 60.0;

        // Calculate the distance to the target
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double d = Math.sqrt(dx * dx + dy * dy);

        // Calculate height difference
        double deltaH = targetPose.getZ() - shooterHeight;

        // This is an anonymous function for use with the numerical solver
        Function<Double, Double> equation = (theta) -> {
            double thetaRad = Math.toRadians(theta);
            return deltaH - (d * Math.tan(thetaRad)) +
                    ((g * d * d) / (2 * v0 * v0 * Math.cos(thetaRad) * Math.cos(thetaRad)));
        };

        // Solve the equation using a numerical method called bisection
        double angle = findRoot(equation, 0, 90);  // Finding root between 0 and 90 degrees

        return angle;
    }

    /**
     * Calculate the angle of the shooter to pass to a location
     * @param robotPose The current {@link Pose2d} of the robot
     * @param targetPose The {@link Pose2d} of the target
     * @param shooterRPM The RPM of the shooter
     * @param wheelDiameter The diameter of the shooter wheel
     * @param shooterHeight The height of the shooter
     * @param minHeight The minimum height of the shot (generally, height of stage)
     * @return The angle of the shooter
     */
    public static double calculateShooterAngleForPass(
            Pose2d robotPose, Pose2d targetPose,
            double shooterRPM, double wheelDiameter,
            double shooterHeight, double minHeight) {

        // Constants
        double g = 9.81;  // Gravity in m/s^2

        // Calculate initial velocity
        double v0 = (shooterRPM * Math.PI * wheelDiameter) / 60.0;

        // Calculate the horizontal distance to the target
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double d = Math.sqrt(dx * dx + dy * dy);

        // Define a function to solve for the angle θ
        Function<Double, Double> equation = (theta) -> {
            double thetaRad = Math.toRadians(theta);

            // Calculate the time to reach the target horizontally
            double t = d / (v0 * Math.cos(thetaRad));

            // Calculate the height of the projectile at that time
            double heightAtTarget = shooterHeight + (v0 * t * Math.sin(thetaRad)) - (0.5 * g * t * t);

            // Ensure the trajectory exceeds the minimum height
            double tAtMinHeight = v0 * Math.sin(thetaRad) / g;  // Time to reach the peak height
            double heightAtMin = shooterHeight + (v0 * tAtMinHeight * Math.sin(thetaRad)) - (0.5 * g * tAtMinHeight * tAtMinHeight);

            if (heightAtMin < minHeight) {
                return minHeight - heightAtMin;  // The difference from the desired minimum height
            } else {
                return heightAtTarget;  // We want this to be 0 (i.e., hitting the target at ground level)
            }
        };

        // Use the bisection method to find the angle θ that ensures the conditions are met
        double angle = findRoot(equation, 0, 90);

        return angle;
    }

    /**
     * Find the root of a function using the bisection method
     * @param equation The function to solve
     * @param min The minimum value to search
     * @param max The maximum value to search
     * @return The root of the function
     */
    public static double findRoot(Function<Double, Double> equation, double min, double max) {
        double tolerance = 1e-6;
        while (max - min > tolerance) {
            double mid = (min + max) / 2;
            if (equation.apply(mid) * equation.apply(min) < 0) {
                max = mid;
            } else {
                min = mid;
            }
        }
        return (min + max) / 2;
    }
}
