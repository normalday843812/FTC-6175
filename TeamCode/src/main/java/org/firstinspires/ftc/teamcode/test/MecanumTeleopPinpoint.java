package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(name="New Teleop", group="TeleOp")
public class MecanumTeleopPinpoint extends LinearOpMode {

    // Drive mode constants
    private static final int FIELD_CENTRIC = 1;
    private static final int ROBOT_CENTRIC_CORRECTED = 2;
    private static final int ROBOT_CENTRIC_BASIC = 3;

    // Set default drive mode here
    private static final int DEFAULT_DRIVE_MODE = FIELD_CENTRIC;

    private int currentDriveMode = DEFAULT_DRIVE_MODE;
    private GoBildaPinpointDriver pinpoint;
    private boolean pinpointInitialized = false;

    // PID correction constants for robot-centric corrected mode
    private static final double P_CORRECTION = 0.03;
    private static final double MAX_CORRECTION = 0.3;

    // Button debouncing
    private boolean lastStickButtonState = false;
    private long lastModeToggleTime = 0;
    private static final long DEBOUNCE_TIME = 250; // milliseconds

    // Hang system constants and variables
    private static final double HANG_MOTOR_SPEED = 1.0;
    private static final long UP_MOVEMENT_TIME = 1000; // 1 second for up movement
    private boolean isGoingUp = false;
    private boolean isGoingDown = false;
    private long upStartTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Mecanum
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Hang motors
        DcMotor hangMotorR = hardwareMap.get(DcMotor.class, "hangMotorR");
        DcMotor hangMotorL = hardwareMap.get(DcMotor.class, "hangMotorL");

        // Misc motors
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        DcMotor scoreMotor = hardwareMap.get(DcMotor.class, "scoreMotor");

        // Intake Servos
        CRServo intakeServo0 = hardwareMap.get(CRServo.class, "intakeServo0");
        CRServo intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        Servo scoreServo = hardwareMap.get(Servo.class, "scoreServo");

        // Motor configurations
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor modes
        DcMotor[] allMotors = {
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor,
                hangMotorL, hangMotorR, intakeMotor, scoreMotor
        };

        for (DcMotor motor : allMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize Pinpoint if needed
        if (currentDriveMode != ROBOT_CENTRIC_BASIC) {
            initializePinpoint();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Mode toggling with both stick buttons
            boolean bothStickButtons = gamepad1.left_stick_button && gamepad1.right_stick_button;
            long currentTime = System.currentTimeMillis();

            if (bothStickButtons && !lastStickButtonState &&
                    (currentTime - lastModeToggleTime) > DEBOUNCE_TIME) {
                currentDriveMode = (currentDriveMode % 3) + 1;
                lastModeToggleTime = currentTime;

                if (currentDriveMode != ROBOT_CENTRIC_BASIC && !pinpointInitialized) {
                    initializePinpoint();
                }
            }
            lastStickButtonState = bothStickButtons;

            // Reset field position with options button
            if (gamepad1.options) {
                if (pinpointInitialized) {
                    pinpoint.resetPosAndIMU();
                    telemetry.addData("Status", "Position Reset!");
                    telemetry.update();
                }
            }

            // Get basic joystick inputs
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // Calculate and apply drive powers
            double[] powers = calculateDrivePowers(x, y, rx);
            frontLeftMotor.setPower(powers[0]);
            backLeftMotor.setPower(powers[1]);
            frontRightMotor.setPower(powers[2]);
            backRightMotor.setPower(powers[3]);

            // Hang system control
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                // Both bumpers - stop all motion
                isGoingUp = false;
                isGoingDown = false;
            } else if (gamepad1.left_bumper && !isGoingUp) {
                // Start upward motion for 1 second
                isGoingUp = true;
                isGoingDown = false;
                upStartTime = System.currentTimeMillis();
            } else if (gamepad1.right_bumper) {
                // Continuous downward motion
                isGoingDown = true;
                isGoingUp = false;
            }

            // Check if we need to stop upward motion after 1 second
            if (isGoingUp && (System.currentTimeMillis() - upStartTime >= UP_MOVEMENT_TIME)) {
                isGoingUp = false;
            }

            // Set motor power based on states
            double hangPower = 0;
            if (isGoingUp) {
                hangPower = HANG_MOTOR_SPEED;
            } else if (isGoingDown) {
                hangPower = -HANG_MOTOR_SPEED;
            }

            // Update hang motors
            hangMotorL.setPower(-hangPower);
            hangMotorR.setPower(hangPower);

            // Score motor
            double scorePower = 0;
            double SCORE_MOTOR_SPEED = 1.0;
            if (gamepad2.right_trigger > 0.5) {
                scorePower = SCORE_MOTOR_SPEED;
            } else if (gamepad2.left_trigger > 0.5) {
                scorePower = -SCORE_MOTOR_SPEED;
            }
            scoreMotor.setPower(scorePower);

            // Intake system control
            double intakePower;
            double INTAKE_POWER_SPEED = 0.25;
            if (gamepad2.left_bumper) {
                intakePower = INTAKE_POWER_SPEED;
            } else if (gamepad2.right_bumper) {
                intakePower = -INTAKE_POWER_SPEED;
            } else {
                intakePower = 0;
            }
            intakeMotor.setPower(intakePower);

            // Intake servo control
            double servoPower = gamepad2.right_stick_y;
            intakeServo0.setPower(servoPower);
            intakeServo1.setPower(-servoPower);

            // Score servo control
            if (gamepad2.a) {
                scoreServo.setPosition(1);
            } else if (gamepad2.b) {
                scoreServo.setPosition(0);
            }

            // Update telemetry
            updateTelemetry();
        }
    }

    private void initializePinpoint() {
        telemetry.addData("Status", "Initializing Pinpoint...");
        telemetry.update();

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.resetPosAndIMU();
            pinpointInitialized = true;

            telemetry.addData("Status", "Pinpoint Initialized Successfully");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Status", "Failed to initialize Pinpoint");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            pinpointInitialized = false;
        }
    }

    private double[] calculateDrivePowers(double x, double y, double rx) {
        double[] powers = new double[4];

        switch (currentDriveMode) {
            case FIELD_CENTRIC:
                if (pinpointInitialized) {
                    // Get robot's heading from Pinpoint
                    double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

                    // Rotate the movement vector by the negative of the heading
                    double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                    double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

                    // Calculate field-centric powers
                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    powers[0] = (rotY + rotX + rx) / denominator; // Front Left
                    powers[1] = (rotY - rotX + rx) / denominator; // Back Left
                    powers[2] = (rotY - rotX - rx) / denominator; // Front Right
                    powers[3] = (rotY + rotX - rx) / denominator; // Back Right
                } else {
                    // Fallback to robot-centric if Pinpoint fails
                    return calculateRobotCentricPowers(x, y, rx, false);
                }
                break;

            case ROBOT_CENTRIC_CORRECTED:
                return calculateRobotCentricPowers(x, y, rx, true);

            case ROBOT_CENTRIC_BASIC:
            default:
                return calculateRobotCentricPowers(x, y, rx, false);
        }

        return powers;
    }

    private double[] calculateRobotCentricPowers(double x, double y, double rx, boolean useCorrection) {
        double[] powers = new double[4];

        if (useCorrection && pinpointInitialized) {
            // Get velocity heading from Pinpoint for drift correction
            double velocityHeading = pinpoint.getVelocity().getHeading(AngleUnit.RADIANS);

            // Calculate correction factor based on unwanted rotation
            double correction = velocityHeading * P_CORRECTION;
            correction = Math.min(Math.max(correction, -MAX_CORRECTION), MAX_CORRECTION);

            // Apply correction to rotation component
            rx += correction;
        }

        // Standard robot-centric mecanum calculation
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        powers[0] = (y + x + rx) / denominator; // Front Left
        powers[1] = (y - x + rx) / denominator; // Back Left
        powers[2] = (y - x - rx) / denominator; // Front Right
        powers[3] = (y + x - rx) / denominator; // Back Right

        return powers;
    }

    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        telemetry.addLine("=== Drive Mode ===");
        switch (currentDriveMode) {
            case FIELD_CENTRIC:
                telemetry.addData("Current Mode", "Field Centric");
                break;
            case ROBOT_CENTRIC_CORRECTED:
                telemetry.addData("Current Mode", "Robot Centric (Corrected)");
                break;
            case ROBOT_CENTRIC_BASIC:
                telemetry.addData("Current Mode", "Robot Centric (Basic)");
                break;
        }

        // Add hang system telemetry
        telemetry.addLine("=== Hang System ===");
        telemetry.addData("Up State", isGoingUp ? "Running" : "Stopped");
        telemetry.addData("Down State", isGoingDown ? "Running" : "Stopped");

        if (pinpointInitialized && currentDriveMode != ROBOT_CENTRIC_BASIC) {
            pinpoint.update();
            Pose2D position = pinpoint.getPosition();
            Pose2D velocity = pinpoint.getVelocity();

            telemetry.addLine("=== Pinpoint Data ===");
            telemetry.addData("Position (mm)", String.format("X: %.1f, Y: %.1f",
                    position.getX(DistanceUnit.MM), position.getY(DistanceUnit.MM)));
            telemetry.addData("Heading (deg)", Math.toDegrees(position.getHeading(AngleUnit.RADIANS)));
            telemetry.addData("Velocity (mm/s)", String.format("X: %.1f, Y: %.1f",
                    velocity.getX(DistanceUnit.MM), velocity.getY(DistanceUnit.MM)));
        }

        telemetry.addLine("\nControls:");
        telemetry.addLine("Press both stick buttons to cycle drive modes");
        telemetry.addLine("Left Bumper: Up for 1 second");
        telemetry.addLine("Right Bumper: Down continuously");
        telemetry.addLine("Both Bumpers: Stop motion");
        telemetry.update();
    }
}