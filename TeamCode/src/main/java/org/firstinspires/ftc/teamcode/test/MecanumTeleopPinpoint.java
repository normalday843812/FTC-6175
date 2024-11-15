package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

/**
 * MecanumTeleopPinpoint
 * This TeleOp mode allows control of a mecanum-wheeled robot while displaying telemetry data
 * from the goBILDA® Pinpoint Odometry Computer.
 * Ensure that:
 * - The motors are correctly mapped in the Robot Configuration.
 * - The Pinpoint device is named "pinpoint" in the Robot Configuration.
 * - The GoBildaPinpointDriver is correctly implemented and placed in the specified package.
 */

@TeleOp(name="Mecanum Teleop with Pinpoint", group="TeleOp")
public class MecanumTeleopPinpoint extends LinearOpMode {

    // Declare Pinpoint driver
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors from hardware map
        // Declare motor variables
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Reverse right side motors to ensure correct orientation
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initializing Pinpoint...");
        telemetry.update();

        try {
            // Instantiate the GoBildaPinpointDriver directly from the hardware map
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            // Set up the Pinpoint device as needed
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
            // Handle the exception as needed
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Exit if stop was requested during initialization
        if (isStopRequested()) return;

        // Main loop
        while (opModeIsActive()) {
            // ====== Mecanum Drive Control ======
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x * 1.1; // Strafe (with potential scaling)
            double rx = gamepad1.right_stick_x; // Rotation

            // Calculate power for each motor
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ====== Update Pinpoint Data ======
            if (pinpoint != null) {
                pinpoint.update(); // Read new data from Pinpoint

                // Retrieve position and velocity
                Pose2D position = pinpoint.getPosition();
                Pose2D velocity = pinpoint.getVelocity();

                // Retrieve device status
                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();

                // ====== Telemetry Data ======
                telemetry.addLine("=== Pinpoint Odometry Data ===");
                telemetry.addData("Position X (mm)", position.getX(DistanceUnit.MM));
                telemetry.addData("Position Y (mm)", position.getY(DistanceUnit.MM));
                telemetry.addData("Heading (rad)", position.getHeading(AngleUnit.RADIANS));
                telemetry.addData("Velocity X (mm/s)", velocity.getX(DistanceUnit.MM));
                telemetry.addData("Velocity Y (mm/s)", velocity.getY(DistanceUnit.MM));
                telemetry.addData("Velocity Heading (rad/s)", velocity.getHeading(AngleUnit.RADIANS));
                telemetry.addData("Device Status", status.toString());
                telemetry.addData("Loop Time (us)", pinpoint.getLoopTime());
                telemetry.addData("Frequency (Hz)", pinpoint.getFrequency());

                // Optional: Display raw encoder values
                telemetry.addData("Raw Encoder X", pinpoint.getEncoderX());
                telemetry.addData("Raw Encoder Y", pinpoint.getEncoderY());

                telemetry.update();
            } else {
                telemetry.addLine("Warning: Pinpoint not initialized");
                telemetry.update();
            }

            // Optional: Add a short delay to reduce loop frequency if necessary
            sleep(10);
        }
    }
}
