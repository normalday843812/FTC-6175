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
 * Main drive class for robot control using Pinpoint localization.
 * Coordinate system:
 * - Positive X is forward
 * - Positive Y is right
 * - Positive heading is clockwise
 */
// if robot drives like ass check DriveController first, not this
@Config
public class PinpointDrive {
    public static class Params {
        // if robot freaks out during comp just bump these up
        public static double MAX_TRAJECTORY_TIME_MULTIPLIER = 1.5;
        public static double MAX_POSITION_ERROR_MM = 50.0;
        public static double MAX_HEADING_ERROR_RAD = Math.PI/6;

        // how many times to try recovering before giving up
        public static int MAX_ERROR_RECOVERY_ATTEMPTS = 3;
        public static double RECOVERY_POSITION_ERROR_MM = 100.0;
        public static double RECOVERY_HEADING_ERROR_RAD = Math.PI/4;
    }

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;

    private final DriveController controller;
    private Pose2d currentPose;
    private final ElapsedTime timer;

    public PinpointDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            // if motors are backwards just flip these
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
            backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Status", "Motors initialized");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Error", "Motors failed: %s", e.getMessage());
            telemetry.update();
            throw new RuntimeException("Failed to initialize motors", e);
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,  // X encoder (forward/back)
                    GoBildaPinpointDriver.EncoderDirection.FORWARD    // Y encoder (left/right)
            );

            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setOffsets(-57.15, 57.15);
            pinpoint.resetPosAndIMU();

            telemetry.addData("Status", "Pinpoint initialized");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Error", "Pinpoint failed: %s", e.getMessage());
            telemetry.update();
            throw new RuntimeException("Failed to initialize Pinpoint", e);
        }

        controller = new DriveController();
        timer = new ElapsedTime();
        updatePoseEstimate();
    }

    public void updatePoseEstimate() {
        try {
            pinpoint.update();
            Pose2D pinpointPose = pinpoint.getPosition();

            currentPose = new Pose2d(
                    pinpointPose.getX(DistanceUnit.MM),
                    pinpointPose.getY(DistanceUnit.MM),
                    pinpointPose.getHeading(AngleUnit.RADIANS)
            );
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to update pose: %s", e.getMessage());
            telemetry.update();
        }
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        // clamp between -1 and 1 because I don't trust myself
        fl = Math.min(Math.max(fl, -1.0), 1.0);
        fr = Math.min(Math.max(fr, -1.0), 1.0);
        bl = Math.min(Math.max(bl, -1.0), 1.0);
        br = Math.min(Math.max(br, -1.0), 1.0);

        try {
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);
        } catch (Exception e) {
            telemetry.addData("Error", "Motor powers failed: %s", e.getMessage());
            telemetry.update();
        }
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
        telemetry.addData("Status", "Motors stopped");
        telemetry.update();
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(currentPose);
    }

    // spent way too long making this not shit itself on error
    public void followTrajectory(Trajectory trajectory) {
        if (trajectory == null) {
            telemetry.addData("Error", "Null trajectory");
            telemetry.update();
            return;
        }

        int recoveryAttempts = 0;
        timer.reset();
        controller.reset();

        double maxTime = trajectory.getTotalTime() * Params.MAX_TRAJECTORY_TIME_MULTIPLIER;

        while (timer.seconds() < trajectory.getTotalTime() && recoveryAttempts < Params.MAX_ERROR_RECOVERY_ATTEMPTS) {
            try {
                updatePoseEstimate();

                if (timer.seconds() > maxTime) {
                    telemetry.addData("Warning", "Timeout, ending trajectory");
                    telemetry.update();
                    break;
                }

                Pose2d targetPose = trajectory.getPose(timer.seconds());
                double positionError = currentPose.position.minus(targetPose.position).norm();
                double headingError = Math.abs(currentPose.getHeadingError(targetPose));

                telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), trajectory.getTotalTime());
                telemetry.addData("Position Error (mm)", "%.2f / %.2f",
                        positionError, Params.MAX_POSITION_ERROR_MM);
                telemetry.addData("Heading Error (deg)", "%.2f / %.2f",
                        Math.toDegrees(headingError), Math.toDegrees(Params.MAX_HEADING_ERROR_RAD));

                // try to recover
                if (positionError > Params.MAX_POSITION_ERROR_MM ||
                        headingError > Params.MAX_HEADING_ERROR_RAD) {

                    recoveryAttempts++;
                    telemetry.addData("Warning", "Error too high, recovery attempt %d/%d",
                            recoveryAttempts, Params.MAX_ERROR_RECOVERY_ATTEMPTS);
                    telemetry.update();

                    double recoveryStartTime = timer.seconds();
                    boolean recovered = false;

                    while (timer.seconds() - recoveryStartTime < 1.0) {
                        updatePoseEstimate();
                        double deltaTime = timer.seconds() - recoveryStartTime;
                        double recoveryScale = Math.min(deltaTime * 2, 0.7);

                        positionError = currentPose.position.minus(targetPose.position).norm();
                        headingError = Math.abs(currentPose.getHeadingError(targetPose));

                        if (positionError <= Params.RECOVERY_POSITION_ERROR_MM &&
                                headingError <= Params.RECOVERY_HEADING_ERROR_RAD) {
                            recovered = true;
                            break;
                        }

                        DriveController.MecanumPowers powers = controller.calculate(currentPose, targetPose);

                        // go slower during recovery so we don't mess up again
                        setMotorPowers(
                                powers.frontLeft * recoveryScale,
                                powers.frontRight * recoveryScale,
                                powers.backLeft * recoveryScale,
                                powers.backRight * recoveryScale
                        );
                    }

                    if (recovered) {
                        telemetry.addData("Status", "Back on track");
                        telemetry.update();
                        continue;
                    }
                }

                DriveController.MecanumPowers powers = controller.calculate(currentPose, targetPose);
                setMotorPowers(
                        powers.frontLeft,
                        powers.frontRight,
                        powers.backLeft,
                        powers.backRight
                );

                telemetry.update();

            } catch (Exception e) {
                telemetry.addData("Error", "Something broke: %s", e.getMessage());
                telemetry.update();
                recoveryAttempts++;
                double errorStartTime = timer.seconds();
                while (timer.seconds() - errorStartTime < 0.5) {
                    updatePoseEstimate();  // Keep updating pose during pause
                }
            }
        }

        // stop smoothly to prevent sliding
        double stopStartTime = timer.seconds();
        while (timer.seconds() - stopStartTime < 0.25) {
            double timeLeft = 0.25 - (timer.seconds() - stopStartTime);
            double scaledPower = Math.max(0.0, timeLeft * 4 * 0.1);
            setMotorPowers(scaledPower, scaledPower, scaledPower, scaledPower);
            updatePoseEstimate();
        }
        stopMotors();

        if (timer.seconds() >= trajectory.getTotalTime()) {
            telemetry.addData("Status", "Done");
        } else {
            telemetry.addData("Warning", "Gave up after %d recovery attempts", recoveryAttempts);
        }
        telemetry.addData("Final Position Error (mm)", "%.2f",
                currentPose.position.minus(trajectory.getEndPose().position).norm());
        telemetry.addData("Final Heading Error (deg)", "%.2f",
                Math.toDegrees(Math.abs(currentPose.getHeadingError(trajectory.getEndPose()))));
        telemetry.update();
    }

    public void turnTo(Vector2d point) {
        Vector2d relative = point.minus(currentPose.position);
        double targetHeading = relative.angle();

        timer.reset();

        while (Math.abs(currentPose.getHeadingError(new Pose2d(currentPose.position, targetHeading))) > Math.toRadians(1)) {
            if (timer.seconds() > 4.0) {
                stopMotors();
                telemetry.addData("Warning", "Turn timed out");
                telemetry.update();
                return;
            }

            updatePoseEstimate();

            double headingError = currentPose.getHeadingError(new Pose2d(currentPose.position, targetHeading));
            telemetry.addData("Heading Error", "%.2f°", Math.toDegrees(headingError));

            double correction = controller.calculate(
                    currentPose,
                    new Pose2d(currentPose.position, targetHeading)
            ).frontLeft;

            setMotorPowers(correction, -correction, correction, -correction);
            telemetry.update();
        }

        // Smooth stop for turns too
        double stopStartTime = timer.seconds();
        while (timer.seconds() - stopStartTime < 0.25) {
            double timeLeft = 0.25 - (timer.seconds() - stopStartTime);
            double scaledPower = Math.max(0.0, timeLeft * 4 * 0.1);
            setMotorPowers(scaledPower, -scaledPower, scaledPower, -scaledPower);
            updatePoseEstimate();
        }
        stopMotors();

        telemetry.addData("Status", "Turn done");
        telemetry.update();
    }

    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    public void setPoseEstimate(Pose2d pose) {
        currentPose = pose;
        telemetry.addData("Reset pose to", "x: %.1f, y: %.1f, θ: %.1f°",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading));
        telemetry.update();
    }

    public double getFrontLeftPower() { return frontLeft.getPower(); }
    public double getFrontRightPower() { return frontRight.getPower(); }
    public double getBackLeftPower() { return backLeft.getPower(); }
    public double getBackRightPower() { return backRight.getPower(); }
}