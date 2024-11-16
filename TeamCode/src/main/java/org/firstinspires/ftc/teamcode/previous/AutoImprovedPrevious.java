package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Autonomous(name="This will probably break")
public class AutoImprovedPrevious extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private static final double POSITION_TOLERANCE = 5.0;
    private static final double HEADING_TOLERANCE = Math.toRadians(2.0);
    private static final double BASE_POWER = 1.0;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        if (opModeIsActive()) {
            strafeToPosition(-457.2); // Left
            strafeToPosition(457.2);  // Right
            driveToPosition(1219.2);  // Forward
            strafeToPosition(381.0);  // Right
        }
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        initializePinpoint();
    }

    private void driveToPosition(double targetDistance) {
        Pose2D startPose = pinpoint.getPosition();
        double targetX = startPose.getX(DistanceUnit.MM) + targetDistance;
        double startHeading = startPose.getHeading(AngleUnit.RADIANS);

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            double xError = targetX - currentPose.getX(DistanceUnit.MM);
            double headingError = normalizeAngle(startHeading - currentPose.getHeading(AngleUnit.RADIANS));

            if (Math.abs(xError) < POSITION_TOLERANCE &&
                    Math.abs(headingError) < HEADING_TOLERANCE) {
                setPowers(0, 0, 0, 0);
                break;
            }

            double power = (xError > 0) ? BASE_POWER : -BASE_POWER;

            double leftPower = power;
            double rightPower = power;
            if (headingError > 0) {
                rightPower *= 0.9;
            } else if (headingError < 0) {
                leftPower *= 0.9;
            }

            setPowers(leftPower, leftPower, rightPower, rightPower);

            telemetry.addData("X Error", "%.2f mm", xError);
            telemetry.addData("Heading Error", "%.2f deg", Math.toDegrees(headingError));
            telemetry.update();
        }
    }

    private void strafeToPosition(double targetDistance) {
        Pose2D startPose = pinpoint.getPosition();
        double targetY = startPose.getY(DistanceUnit.MM) + targetDistance;
        double startHeading = startPose.getHeading(AngleUnit.RADIANS);

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            double yError = targetY - currentPose.getY(DistanceUnit.MM);
            double headingError = normalizeAngle(startHeading - currentPose.getHeading(AngleUnit.RADIANS));

            if (Math.abs(yError) < POSITION_TOLERANCE &&
                    Math.abs(headingError) < HEADING_TOLERANCE) {
                setPowers(0, 0, 0, 0);
                break;
            }

            double power = (yError > 0) ? BASE_POWER : -BASE_POWER;

            double correction = (headingError > 0) ? 0.1 : (headingError < 0) ? -0.1 : 0;

            setPowers(
                    power + correction,
                    -power + correction,
                    -power - correction,
                    power - correction
            );


            telemetry.update();
        }
    }

    private void initializePinpoint() {
        telemetry.addData("Status", "Initializing Pinpoint...");
        telemetry.update();

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.resetPosAndIMU();

            telemetry.addData("Status", "Pinpoint Initialized Successfully");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Status", "Failed to initialize Pinpoint");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void setPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }
}