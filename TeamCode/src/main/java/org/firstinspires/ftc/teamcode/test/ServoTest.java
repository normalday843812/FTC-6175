//package org.firstinspires.ftc.teamcode.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@TeleOp(name = "Servo Test", group = "Test")
//public class ServoTest extends LinearOpMode {
//
//    // Use built-in servo constants for position limits
//    private static final double MIN_POSITION = Servo.MIN_POSITION;  // 0.0
//    private static final double MAX_POSITION = Servo.MAX_POSITION;  // 1.0
//    private static final double CENTER_POSITION = (MAX_POSITION - MIN_POSITION) / 2.0;
//
//    // Define our specific positions with smaller range
//    private static final double DOWN_POSITION = 0.45;  // 81 degrees
//    private static final double UP_POSITION = 0.55;    // 99 degrees
//
//    // Add debounce time to prevent rapid button triggers
//    private static final double DEBOUNCE_TIME_MS = 250;
//
//    @Override
//    public void runOpMode() {
//        Servo testServo;
//        ElapsedTime buttonTimer = new ElapsedTime();
//
//        // Initialize the servo
//        try {
//            testServo = hardwareMap.get(Servo.class, "TestServo");
//
//            // Optional: Scale the servo range to limit movement
//            // This will map the full range of motion to only move between 0.4 and 0.6
//            testServo.scaleRange(DOWN_POSITION, UP_POSITION);
//
//            // Initialize servo to center position
//            testServo.setPosition(CENTER_POSITION);
//
//        } catch (Exception e) {
//            telemetry.addData("Error", "Could not find servo named 'TestServo'");
//            telemetry.addData("Check", "Is the servo configured in the Robot Controller app?");
//            telemetry.update();
//            return;
//        }
//
//        // Show controls on Driver Station
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Controls", "A: Up Position (%.2f)", UP_POSITION);
//        telemetry.addData("Controls", "B: Down Position (%.2f)", DOWN_POSITION);
//        telemetry.addData("Controls", "Y: Center Position (%.2f)", CENTER_POSITION);
//        telemetry.update();
//
//        waitForStart();
//
//        double currentPosition = CENTER_POSITION;
//        boolean lastAState = false;
//        boolean lastBState = false;
//        boolean lastYState = false;
//
//        while (opModeIsActive()) {
//            boolean positionChanged = false;
//
//            // Check for button presses with debouncing
//            if (buttonTimer.milliseconds() > DEBOUNCE_TIME_MS) {
//                if (gamepad1.a && !lastAState) {
//                    currentPosition = 1.0;  // Will map to UP_POSITION due to scaleRange
//                    positionChanged = true;
//                    buttonTimer.reset();
//                }
//                else if (gamepad1.b && !lastBState) {
//                    currentPosition = 0.0;  // Will map to DOWN_POSITION due to scaleRange
//                    positionChanged = true;
//                    buttonTimer.reset();
//                }
//                else if (gamepad1.y && !lastYState) {
//                    currentPosition = 0.5;  // Will map to CENTER_POSITION due to scaleRange
//                    positionChanged = true;
//                    buttonTimer.reset();
//                }
//            }
//
//            // Update last button states
//            lastAState = gamepad1.a;
//            lastBState = gamepad1.b;
//            lastYState = gamepad1.y;
//
//            // Only set position if it changed (reduces unnecessary commands)
//            if (positionChanged) {
//                testServo.setPosition(currentPosition);
//            }
//
//            // Update telemetry
//            telemetry.addData("Servo Position", "%.3f", testServo.getPosition());
//            telemetry.addData("Raw Target", "%.3f", currentPosition);
//            telemetry.addData("Status", "Running");
//            telemetry.update();
//        }
//    }
//}