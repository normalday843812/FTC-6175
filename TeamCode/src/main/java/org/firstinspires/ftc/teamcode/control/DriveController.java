// if something fucks up it's probably this file
package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;

@Config
public class DriveController {
    public static class Params {
        // Position control gains
        public static double X_kP = 0.1;
        public static double Y_kP = 0.1;
        public static double HEADING_kP = 0.1;

        // Velocity control gains
        public static double VEL_kP = 0.1;
        public static double VEL_kI = 0.0;
        public static double VEL_kD = 0.0;

        // Feedforward gains
        public static double kS = 0.0;  // Static friction
        public static double kV = 0.0;  // Velocity feedforward
        public static double kA = 0.0;  // Acceleration feedforward
    }

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    private final PIDController velocityController;

    public DriveController() {
        xController = new PIDController(Params.X_kP, 0, 0);
        yController = new PIDController(Params.Y_kP, 0, 0);
        headingController = new PIDController(Params.HEADING_kP, 0, 0);
        velocityController = new PIDController(Params.VEL_kP, Params.VEL_kI, Params.VEL_kD);

        // Limit outputs to [-1, 1] for motor powers
        xController.setOutputLimits(-1, 1);
        yController.setOutputLimits(-1, 1);
        headingController.setOutputLimits(-1, 1);
        velocityController.setOutputLimits(-1, 1);
    }

    /**
     * Calculate mecanum drive powers to reach target pose
     * Coordinate system:
     * - Positive X is forward
     * - Positive Y is right
     * - Positive heading is clockwise
     */
    public MecanumPowers calculate(Pose2d currentPose, Pose2d targetPose) {
        // Calculate errors in robot frame
        Pose2d error = currentPose.getError(targetPose);

        // Transform to robot-centric frame
        Vector2d fieldError = error.position;
        Vector2d robotError = fieldError.rotated(+currentPose.heading); // Changed from negative to positive rotation

        // Calculate individual axis corrections
        double xCorrection = xController.calculate(robotError.x);
        double yCorrection = yController.calculate(robotError.y);
        double headingCorrection = headingController.calculate(error.heading);

        // Convert to mecanum powers
        return new MecanumPowers(
                xCorrection,
                yCorrection,
                headingCorrection
        );
    }

    public void reset() {
        xController.reset();
        yController.reset();
        headingController.reset();
        velocityController.reset();
    }

    public static class MecanumPowers {
        public final double frontLeft;
        public final double frontRight;
        public final double backLeft;
        public final double backRight;

        public MecanumPowers(double x, double y, double heading) {
            // Convert x, y, heading to mecanum wheel powers
            // For standard mecanum configuration:
            // - Positive X is forward
            // - Positive Y is right
            // - Positive heading is clockwise
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(heading), 1);

            // FIXED mecanum formulas for standard configuration
            frontLeft = (y - x - heading) / denominator;    // Changed signs
            backLeft = (y + x - heading) / denominator;     // Changed signs
            frontRight = (y + x + heading) / denominator;   // Changed signs
            backRight = (y - x + heading) / denominator;    // Changed signs
        }

        @Override
        public String toString() {
            return String.format("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                    frontLeft, frontRight, backLeft, backRight);
        }
    }
}