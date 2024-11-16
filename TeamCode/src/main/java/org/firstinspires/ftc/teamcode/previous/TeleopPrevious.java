//package org.firstinspires.ftc.teamcode.previous;
//
//import android.annotation.SuppressLint;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import com.qualcomm.ftccommon.SoundPlayer;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
//import org.firstinspires.ftc.teamcode.RobotHardware;
//
//@TeleOp(name="Choose this for teleop", group="Linear Opmode")
//public class TeleopPrevious extends LinearOpMode {
//    private boolean robotCentric = false;
//    private boolean isRotating = false;
//    private static final double ROTATION_POWER = 0.5;
//    private static final double ROTATION_TOLERANCE = 3.0; // degrees
//
//    public static double sigmoid(double x) {
//        return 1 / (1 + Math.exp(-x));
//    }
//    private void perform180Turn(RobotHardware robot, IMU imu) {
//        // Set motors to run using encoders for better control
//        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Get current heading and calculate target (180° from current)
//        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        double targetYaw = currentYaw + 180;
//        if (targetYaw > 180) targetYaw -= 360;
//        isRotating = true;
//
//        while (isRotating && opModeIsActive()) {
//            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            double error = targetYaw - currentYaw;
//
//            // Normalize error to [-180, 180]
//            while (error > 180) error -= 360;
//            while (error < -180) error += 360;
//
//            // Check if we're within tolerance
//            if (Math.abs(error) < ROTATION_TOLERANCE) {
//                robot.frontLeftMotor.setPower(0);
//                robot.frontRightMotor.setPower(0);
//                robot.backLeftMotor.setPower(0);
//                robot.backRightMotor.setPower(0);
//                break;
//            }
//
//            // Calculate power based on error
//            double power = Math.signum(error) * ROTATION_POWER;
//            if (Math.abs(error) < 45) { // Reduce power when close to target
//                power *= Math.abs(error) / 45.0;
//            }
//
//            // Set motor powers for rotation
//            robot.frontLeftMotor.setPower(power);
//            robot.backLeftMotor.setPower(power);
//            robot.frontRightMotor.setPower(-power);
//            robot.backRightMotor.setPower(-power);
//
//            // Update telemetry
//            telemetry.addData("Current Yaw", currentYaw);
//            telemetry.addData("Target Yaw", targetYaw);
//            telemetry.addData("Error", error);
//            telemetry.update();
//        }
//
//        // Reset motor modes
//        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    @SuppressLint("DefaultLocale")
//    @Override
//    public void runOpMode() {
//        RobotHardware robot = new RobotHardware(hardwareMap);
//
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//
//        AndroidSoundPool androidSoundPool = new AndroidSoundPool();
//        androidSoundPool.initialize(SoundPlayer.getInstance());
//        androidSoundPool.preloadSound("fun.mp3");
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            if (!isRotating) {
//                double y = gamepad1.left_stick_y;
//                double x = gamepad1.left_stick_x * -1.1;
//                double rx = gamepad1.right_stick_x;
//
//                if (gamepad1.options) {
//                    imu.resetYaw();
//                }
//
//                if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
//                    robotCentric = !robotCentric;
//                }
//
//                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//                double rotX, rotY;
//                if (robotCentric) {
//                    rotX = x;
//                    rotY = y;
//                } else {
//                    rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//                    rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//                }
//
//                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//                double frontLeftPower = (rotY + rotX + rx) / denominator;
//                double backLeftPower = (rotY - rotX + rx) / denominator;
//                double frontRightPower = (rotY - rotX - rx) / denominator;
//                double backRightPower = (rotY + rotX - rx) / denominator;
//
//                robot.frontLeftMotor.setPower(frontLeftPower);
//                robot.backLeftMotor.setPower(backLeftPower);
//                robot.frontRightMotor.setPower(frontRightPower);
//                robot.backRightMotor.setPower(backRightPower);
//
//                // Rest of the controls remain the same
//                double intakeArmPower = 0;
//                double INTAKE_ARM_SPEED = 1.0;
//                if (gamepad2.left_trigger > 0.1) {
//                    intakeArmPower = -INTAKE_ARM_SPEED * gamepad2.left_trigger;
//                } else if (gamepad2.right_trigger > 0.1) {
//                    intakeArmPower = INTAKE_ARM_SPEED * gamepad2.right_trigger;
//                }
//                robot.intakeMotor.setPower(intakeArmPower);
//
//                double servoPower = gamepad2.right_stick_y;
//                robot.intakeServo1.setPower(servoPower);
//                robot.intakeServo2.setPower(-servoPower);
//
//                double SCORE_ARM_SPEED = 1;
//                if (gamepad2.a) {
//                    robot.scoreMotor.setPower(SCORE_ARM_SPEED);
//                } else if (gamepad2.b) {
//                    robot.scoreMotor.setPower(-SCORE_ARM_SPEED);
//                }
//
//                double hangPower = 0;
//                double HANG_MOTOR_SPEED = 1.0;
//                if (gamepad1.left_bumper) {
//                    hangPower = HANG_MOTOR_SPEED;
//                } else if (gamepad1.right_bumper) {
//                    hangPower = -HANG_MOTOR_SPEED;
//                }
//
//                robot.hangMotorL.setPower(hangPower);
//                robot.hangMotorR.setPower(-hangPower);
//
//                if (gamepad2.dpad_left) {
//                    robot.scoreServo.setPosition(1);
//                } else if (gamepad2.dpad_right) {
//                    robot.scoreServo.setPosition(0);
//                }
//
//                if (gamepad2.dpad_down) {
//                    robot.scorePivotServo.setPosition(1);
//                } else if (gamepad2.dpad_up) {
//                    robot.scorePivotServo.setPosition(0);
//                }
//
//                // === DRIVE SYSTEM ===
//                telemetry.addLine("╔══════════ DRIVE SYSTEM ══════════");
//                telemetry.addData("║ Drive Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
//                telemetry.addData("║ Front Left Motor", String.format("Power: %.3f, Pos: %d",
//                        robot.frontLeftMotor.getPower(), robot.frontLeftMotor.getCurrentPosition()));
//                telemetry.addData("║ Front Right Motor", String.format("Power: %.3f, Pos: %d",
//                        robot.frontRightMotor.getPower(), robot.frontRightMotor.getCurrentPosition()));
//                telemetry.addData("║ Back Left Motor", String.format("Power: %.3f, Pos: %d",
//                        robot.backLeftMotor.getPower(), robot.backLeftMotor.getCurrentPosition()));
//                telemetry.addData("║ Back Right Motor", String.format("Power: %.3f, Pos: %d",
//                        robot.backRightMotor.getPower(), robot.backRightMotor.getCurrentPosition()));
//                telemetry.addLine("║ Motor Modes:");
//                telemetry.addData("║  - Front Left", robot.frontLeftMotor.getMode());
//                telemetry.addData("║  - Front Right", robot.frontRightMotor.getMode());
//                telemetry.addData("║  - Back Left", robot.backLeftMotor.getMode());
//                telemetry.addData("║  - Back Right", robot.backRightMotor.getMode());
//
//// === IMU DATA ===
//                telemetry.addLine("╠══════════ IMU DATA ══════════");
//                telemetry.addData("║ Robot Heading (deg)", "%.2f", Math.toDegrees(botHeading));
//                telemetry.addData("║ Raw IMU Yaw", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//                telemetry.addData("║ Raw IMU Pitch", "%.2f", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
//                telemetry.addData("║ Raw IMU Roll", "%.2f", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
//
//// === INTAKE SYSTEM ===
//                telemetry.addLine("╠══════════ INTAKE SYSTEM ══════════");
//                telemetry.addData("║ Intake Motor", String.format("Power: %.3f, Pos: %d",
//                        robot.intakeMotor.getPower(), robot.intakeMotor.getCurrentPosition()));
//                telemetry.addData("║ Motor Mode", robot.intakeMotor.getMode());
//                telemetry.addData("║ Intake Servo 1", String.format("Power: %.3f", robot.intakeServo1.getPower()));
//                telemetry.addData("║ Intake Servo 2", String.format("Power: %.3f", robot.intakeServo2.getPower()));
//
//// === SCORING SYSTEM ===
//                telemetry.addLine("╠══════════ SCORING SYSTEM ══════════");
//                telemetry.addData("║ Score Motor", String.format("Power: %.3f, Pos: %d",
//                        robot.scoreMotor.getPower(), robot.scoreMotor.getCurrentPosition()));
//                telemetry.addData("║ Target Position", robot.scoreMotor.getTargetPosition());
//                telemetry.addData("║ Motor Mode", robot.scoreMotor.getMode());
//                telemetry.addData("║ Score Servo", String.format("Position: %.3f", robot.scoreServo.getPosition()));
//                telemetry.addData("║ Score Pivot Servo", String.format("Position: %.3f", robot.scorePivotServo.getPosition()));
//
//// === HANGING SYSTEM ===
//                telemetry.addLine("╠══════════ HANGING SYSTEM ══════════");
//                telemetry.addData("║ Left Hang Motor", String.format("Power: %.3f, Pos: %d",
//                        robot.hangMotorL.getPower(), robot.hangMotorL.getCurrentPosition()));
//                telemetry.addData("║ Right Hang Motor", String.format("Power: %.3f, Pos: %d",
//                        robot.hangMotorR.getPower(), robot.hangMotorR.getCurrentPosition()));
//                telemetry.addData("║ Left Motor Mode", robot.hangMotorL.getMode());
//                telemetry.addData("║ Right Motor Mode", robot.hangMotorR.getMode());
//
//// === SYSTEM STATUS ===
//                telemetry.addLine("╠══════════ SYSTEM STATUS ══════════");
//                telemetry.addData("║ OpMode Active", opModeIsActive());
//                telemetry.addData("║ Is Rotating", isRotating);
//                telemetry.addData("║ Runtime", String.format("%.2f seconds", getRuntime()));
//
//// === GAMEPAD 1 STATUS ===
//                telemetry.addLine("╠══════════ GAMEPAD 1 ══════════");
//                telemetry.addData("║ Left Stick", String.format("X: %.4f, Y: %.4f", gamepad1.left_stick_x, gamepad1.left_stick_y));
//                telemetry.addData("║ Right Stick", String.format("X: %.4f, Y: %.4f", gamepad1.right_stick_x, gamepad1.right_stick_y));
//                telemetry.addData("║ DPad", String.format("Up: %b, Down: %b, Left: %b, Right: %b",
//                        gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right));
//                telemetry.addData("║ Buttons", String.format("A: %b, B: %b, X: %b, Y: %b",
//                        gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y));
//                telemetry.addData("║ Bumpers", String.format("Left: %b, Right: %b", gamepad1.left_bumper, gamepad1.right_bumper));
//                telemetry.addData("║ Triggers", String.format("Left: %.2f, Right: %.2f", gamepad1.left_trigger, gamepad1.right_trigger));
//                telemetry.addData("║ Other", String.format("Back: %b, Start: %b, Guide: %b",
//                        gamepad1.back, gamepad1.start, gamepad1.guide));
//                telemetry.addData("║ Stick Buttons", String.format("Left: %b, Right: %b",
//                        gamepad1.left_stick_button, gamepad1.right_stick_button));
//
//// === GAMEPAD 2 STATUS ===
//                telemetry.addLine("╠══════════ GAMEPAD 2 ══════════");
//                telemetry.addData("║ Left Stick", String.format("X: %.4f, Y: %.4f", gamepad2.left_stick_x, gamepad2.left_stick_y));
//                telemetry.addData("║ Right Stick", String.format("X: %.4f, Y: %.4f", gamepad2.right_stick_x, gamepad2.right_stick_y));
//                telemetry.addData("║ DPad", String.format("Up: %b, Down: %b, Left: %b, Right: %b",
//                        gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right));
//                telemetry.addData("║ Buttons", String.format("A: %b, B: %b, X: %b, Y: %b",
//                        gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y));
//                telemetry.addData("║ Bumpers", String.format("Left: %b, Right: %b", gamepad2.left_bumper, gamepad2.right_bumper));
//                telemetry.addData("║ Triggers", String.format("Left: %.2f, Right: %.2f", gamepad2.left_trigger, gamepad2.right_trigger));
//                telemetry.addData("║ Other", String.format("Back: %b, Start: %b, Guide: %b",
//                        gamepad2.back, gamepad2.start, gamepad2.guide));
//                telemetry.addData("║ Stick Buttons", String.format("Left: %b, Right: %b",
//                        gamepad2.left_stick_button, gamepad2.right_stick_button));
//
//                telemetry.addLine("╚══════════════════════════════");
//                telemetry.update();
//            }
//        }
//
//        androidSoundPool.close();
//    }
//}