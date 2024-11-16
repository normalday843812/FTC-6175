package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;

import androidx.annotation.NonNull;
import java.util.Locale;

// if the robot still drives like ass after tuning these:
// 1. check if your motors are actually mapped right
// 2. verify coordinate system matches reality
// 3. make sure pinpoint's working
// if all that's good then yeah it's probably these values
@Config
public class DriveController {
    public static class Params {
        // Start with these during comp, adjust if robot is:
        // - Sluggish: increase kP
        // - Wobbly: decrease kP or increase kD
        // - Not hitting target: increase kI
        // - Overshooting: decrease kP or increase kD
        public static double X_kP = 0.8;
        public static double X_kI = 0.1;
        public static double X_kD = 0.2;

        public static double Y_kP = 0.8;
        public static double Y_kI = 0.1;
        public static double Y_kD = 0.2;

        public static double HEADING_kP = 1.2;
        public static double HEADING_kI = 0.15;
        public static double HEADING_kD = 0.3;

        // Feedforward for better speed control
        public static double kS = 0.1;  // Static friction compensation
        public static double kV = 1.0 / 1000.0;  // Velocity feedforward (1/max velocity)

        // For competition emergencies
        public static boolean DISABLE_HEADING_CONTROL = false;
        public static double POWER_SCALE = 1.0;  // Set < 1 to reduce all motor powers
    }

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    private final PIDController velocityController;

    public DriveController() {
        xController = new PIDController(Params.X_kP, Params.X_kI, Params.X_kD);
        yController = new PIDController(Params.Y_kP, Params.Y_kI, Params.Y_kD);
        headingController = new PIDController(Params.HEADING_kP, Params.HEADING_kI, Params.HEADING_kD);
        velocityController = new PIDController(Params.kV, 0, 0);

        // Prevent integral windup
        xController.setIntegralLimits(-0.5, 0.5);
        yController.setIntegralLimits(-0.5, 0.5);
        headingController.setIntegralLimits(-0.5, 0.5);

        // Limit outputs to [-1, 1] for motor powers
        xController.setOutputLimits(-1, 1);
        yController.setOutputLimits(-1, 1);
        headingController.setOutputLimits(-1, 1);
        velocityController.setOutputLimits(-1, 1);
    }

    public void reset() {
        xController.reset();
        yController.reset();
        headingController.reset();
        velocityController.reset();
    }

    /**
     * Calculate mecanum drive powers to reach target pose
     */
    public MecanumPowers calculate(Pose2d currentPose, Pose2d targetPose) {
        // Calculate errors in robot frame
        Pose2d error = currentPose.getError(targetPose);
        Vector2d fieldError = error.position;
        Vector2d robotError = fieldError.rotated(+currentPose.heading);  // +heading for our coordinate system

        // Calculate velocities (for feedforward)
        double targetVelocity = robotError.norm() * Params.kV;
        double staticCompensation = Math.signum(targetVelocity) * Params.kS;

        // Calculate PID outputs
        double xCorrection = xController.calculate(robotError.x);
        double yCorrection = yController.calculate(robotError.y);

        // Add feedforward
        xCorrection += Math.signum(xCorrection) * staticCompensation;
        yCorrection += Math.signum(yCorrection) * staticCompensation;

        // Calculate heading correction (can be disabled in emergency)
        double headingCorrection = Params.DISABLE_HEADING_CONTROL ?
                0.0 : headingController.calculate(error.heading);

        // Scale everything if needed (for testing or emergency)
        xCorrection *= Params.POWER_SCALE;
        yCorrection *= Params.POWER_SCALE;
        headingCorrection *= Params.POWER_SCALE;

        // Convert to mecanum powers
        return new MecanumPowers(
                xCorrection,
                yCorrection,
                headingCorrection
        );
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

            frontLeft = (y - x - heading) / denominator;
            backLeft = (y + x - heading) / denominator;
            frontRight = (y + x + heading) / denominator;
            backRight = (y - x + heading) / denominator;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.US, "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                    frontLeft, frontRight, backLeft, backRight);
        }
    }
}