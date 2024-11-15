package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.DriveController;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.trajectories.TrajectoryBuilder;

/**
 * Robot drive system using GoBilda Pinpoint thing
 * Coordinate system:
 * - Positive X is forward
 * - Positive Y is right
 * - Positive heading is clockwise
 * Motor configuration:
 * - Front Left:  forward = forward (FORWARD)
 * - Front Right: forward = forward (REVERSE)
 * - Back Left:   forward = forward (FORWARD)
 * - Back Right:  forward = forward (REVERSE)
 */

@Config
public class PinpointDrive {
    public static class Params {
        public static double MAX_TRAJECTORY_TIME_MULTIPLIER = 1.5;  // Maximum time multiplier before timeout
        public static double MAX_POSITION_ERROR_MM = 50.0;         // Maximum allowable position error in mm
        public static double MAX_HEADING_ERROR_RAD = Math.PI/6;    // Maximum allowable heading error in radians
    }

    // Hardware
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final GoBildaPinpointDriver pinpoint;

    private final DriveController controller;
    private Pose2d currentPose;
    private final ElapsedTime timer;

    public PinpointDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize motors
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
            backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

            // Set motor directions
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Initialize Pinpoint
            try {
                pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

                // TODO: If position tracking is mirrored, flip these encoder directions
                pinpoint.setEncoderDirections(
                        GoBildaPinpointDriver.EncoderDirection.REVERSED,  // X encoder (forward/back)
                        GoBildaPinpointDriver.EncoderDirection.FORWARD   // Y encoder (left/right)
                );

                pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

                pinpoint.resetPosAndIMU();
                telemetry.addData("Status", "Initialized Successfully");
                telemetry.update();
            } catch (Exception e) {
                telemetry.addData("Status", "Failed to initialize Pinpoint");
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
                throw new RuntimeException("Failed to initialize Pinpoint", e);
            }

            controller = new DriveController();
            timer = new ElapsedTime();
            updatePoseEstimate();

        } catch (Exception e) {
            telemetry.addData("Status", "Failed to initialize drive system");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            throw new RuntimeException("Failed to initialize drive system", e);
        }
    }

    public void updatePoseEstimate() {
        pinpoint.update();
        Pose2D pinpointPose = pinpoint.getPosition();

        currentPose = new Pose2d(
                pinpointPose.getX(DistanceUnit.MM),
                pinpointPose.getY(DistanceUnit.MM),
                pinpointPose.getHeading(AngleUnit.RADIANS)
        );
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        fl = Math.min(Math.max(fl, -1.0), 1.0); // self explanatory
        fr = Math.min(Math.max(fr, -1.0), 1.0);
        bl = Math.min(Math.max(bl, -1.0), 1.0);
        br = Math.min(Math.max(br, -1.0), 1.0);

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(currentPose);
    }

    public void followTrajectory(Trajectory trajectory) {
        if (trajectory == null) {
            return;
        }

        timer.reset();
        controller.reset();

        double maxTime = trajectory.getTotalTime() * Params.MAX_TRAJECTORY_TIME_MULTIPLIER;

        while (timer.seconds() < trajectory.getTotalTime()) {
            // Check for timeout
            if (timer.seconds() > maxTime) {
                stopMotors();
                return;
            }

            updatePoseEstimate();

            // Get target pose from trajectory
            double currentTime = timer.seconds();
            Pose2d targetPose = trajectory.getPose(currentTime);

            // Check position error
            double positionError = currentPose.position.minus(targetPose.position).norm();
            double headingError = Math.abs(currentPose.getHeadingError(targetPose));

            if (positionError > Params.MAX_POSITION_ERROR_MM ||
                    headingError > Params.MAX_HEADING_ERROR_RAD) {
                stopMotors();
                return;
            }

            // Calculate and apply motor powers
            DriveController.MecanumPowers powers = controller.calculate(currentPose, targetPose);
            setMotorPowers(
                    powers.frontLeft,
                    powers.frontRight,
                    powers.backLeft,
                    powers.backRight
            );
        }

        stopMotors();
    }

    public void turnTo(Vector2d point) {
        Vector2d relative = point.minus(currentPose.position);
        double targetHeading = relative.angle();

        timer.reset();
        while (Math.abs(currentPose.getHeadingError(new Pose2d(currentPose.position, targetHeading))) > Math.toRadians(1)) {
            if (timer.seconds() > 4.0) {  // 4 second timeout for turns
                stopMotors();
                return;
            }

            updatePoseEstimate();

            double correction = controller.calculate(
                    currentPose,
                    new Pose2d(currentPose.position, targetHeading)
            ).frontLeft;

            setMotorPowers(correction, -correction, correction, -correction);
        }

        stopMotors();
    }

    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    public void setPoseEstimate(Pose2d pose) {
        currentPose = pose;
    }

    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }
    public double getFrontRightPower() {
        return frontRight.getPower();
    }
    public double getBackLeftPower() {
        return backLeft.getPower();
    }
    public double getBackRightPower() {
        return backRight.getPower();
    }
}