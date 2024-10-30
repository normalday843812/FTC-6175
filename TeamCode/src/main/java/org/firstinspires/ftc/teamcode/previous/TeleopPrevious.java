package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.ftccommon.SoundPlayer;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;

@TeleOp(name="teleop", group="Linear Opmode")
public class TeleopPrevious extends LinearOpMode {

    private boolean robotCentric = false;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor scoreMotor = hardwareMap.dcMotor.get("scoreMotor");
        CRServo intakeServo1 = hardwareMap.crservo.get("intakeServo1");
        CRServo intakeServo2 = hardwareMap.crservo.get("intakeServo2");
        Servo scoreServo = hardwareMap.servo.get("scoreServo");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoreMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configure score motor for position control
        scoreMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set drive motors to run using encoders for field-centric mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        AndroidSoundPool androidSoundPool = new AndroidSoundPool();
        androidSoundPool.initialize(SoundPlayer.getInstance());
        androidSoundPool.preloadSound("fun.mp3");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        androidSoundPool.play("fun.mp3");

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                robotCentric = !robotCentric;
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX, rotY;
            if (robotCentric) {
                rotX = x;
                rotY = y;
            } else {
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            }

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Intake arm control
            double intakeArmPower = 0;
            // Motor control variables
            double INTAKE_ARM_SPEED = 0.25;
            if (gamepad2.left_trigger > 0.1) {
                intakeArmPower = -INTAKE_ARM_SPEED * gamepad2.left_trigger;
            } else if (gamepad2.right_trigger > 0.1) {
                intakeArmPower = INTAKE_ARM_SPEED * gamepad2.right_trigger;
            }
            intakeMotor.setPower(intakeArmPower);

            double servoPower = gamepad2.right_stick_y;
            intakeServo1.setPower(servoPower);
            intakeServo2.setPower(-servoPower);

            // Score motor control - Automatic position control with A/B buttons
            double SCORE_ARM_SPEED = 0.8;
            if (gamepad2.a) {
                // Score motor position constants
                // Adjust this value for desired up position
                int SCORE_MOTOR_UP_POSITION = 1000;
                scoreMotor.setTargetPosition(SCORE_MOTOR_UP_POSITION);
                scoreMotor.setPower(SCORE_ARM_SPEED);
            } else if (gamepad2.b) {
                // Down position (starting position)
                int SCORE_MOTOR_DOWN_POSITION = 0;
                scoreMotor.setTargetPosition(SCORE_MOTOR_DOWN_POSITION);
                scoreMotor.setPower(SCORE_ARM_SPEED);
            }

            // Score servo control
            double currentScorePosition = scoreServo.getPosition();
            if (gamepad2.x) {  // Changed from 'a' to 'x' to avoid conflict with score motor control
                double SCORE_SERVO_MAX = 1.0;
                if (currentScorePosition == SCORE_SERVO_MAX) {
                    // ScoreServo positions
                    double SCORE_SERVO_MIN = 0.0;
                    scoreServo.setPosition(SCORE_SERVO_MIN);
                } else {
                    scoreServo.setPosition(SCORE_SERVO_MAX);
                }
            }

            // Sound control
            if (gamepad1.x) {
                androidSoundPool.play("fun.mp3");
            } else if (gamepad1.y) {
                androidSoundPool.stop();
            }

            telemetry.addData("Drive Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
            telemetry.addData("Intake Arm Power", intakeArmPower);
            telemetry.addData("Score Motor Position", scoreMotor.getCurrentPosition());
            telemetry.addData("Score Motor Target", scoreMotor.getTargetPosition());
            telemetry.addData("Score Servo", scoreServo.getPosition());
            //telemetry.addData("Hang Motors", hangPower);
            telemetry.update();
        }

        androidSoundPool.close();
    }
}
