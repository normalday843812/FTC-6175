package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name="Monkey", group="TeleOp")
public class ClawTest extends LinearOpMode {
    private OpenCvCamera camera;
    private BlueDetectionPipeline pipeline;
    private Servo clawServo;
    private Servo clawRollServo;
    private Servo clawPitchServo;
    private static final double CLAW_MIN_POSITION = 0.14;
    private static final double CLAW_MAX_POSITION = 0.39;
    private static final double DEADZONE = 0.1;
    private static final int BUFFER_SIZE = 50;
    private static final double MAX_SERVO_SPEED = 1.0;
    private boolean isClawOpen = false;
    private boolean previousAState = false;
    private boolean previousDpadUpState = false;
    private boolean previousDpadDownState = false;
    private double clawRollServoPos = 0.5;
    private double clawPitchServoPos = 0.5;
    private final List<Double> angleBuffer = new ArrayList<>();
    private boolean isCollectingData = false;
    private Double finalAngle = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        initializeCamera();

        waitForStart();
        if (isStopRequested()) return;

        long lastUpdateTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
            lastUpdateTime = currentTime;

            handleDataCollection();
            handleServoControls(deltaTime);
            updateTelemetry();

            if (!opModeIsActive()) break;
        }

        cleanup();
    }

    private void initializeHardware() {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawRollServo = hardwareMap.get(Servo.class, "clawRollServo");
        clawPitchServo = hardwareMap.get(Servo.class, "clawPitchServo");
    }

    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        pipeline = new BlueDetectionPipeline();
        camera.setPipeline(pipeline);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    private void handleDataCollection() {
        if (gamepad1.dpad_up && !previousDpadUpState) {
            isCollectingData = true;
            angleBuffer.clear();
            finalAngle = null;
        }

        if (gamepad1.dpad_down && !previousDpadDownState) {
            if (isCollectingData && !angleBuffer.isEmpty()) {
                isCollectingData = false;
                calculateAndSetFinalPosition();
            }
        }

        if (isCollectingData && pipeline.isBlueObjectDetected()) {
            double currentAngle = pipeline.getAngle();
            angleBuffer.add(currentAngle);

            while (angleBuffer.size() > BUFFER_SIZE) {
                angleBuffer.remove(0);
            }

            updateServoToAngle(currentAngle);
        }

        previousDpadUpState = gamepad1.dpad_up;
        previousDpadDownState = gamepad1.dpad_down;
    }

    private void calculateAndSetFinalPosition() {
        double sum = 0;
        int count = 0;
        int startIndex = Math.max(0, angleBuffer.size() - (angleBuffer.size() / 2));

        for (int i = startIndex; i < angleBuffer.size(); i++) {
            sum += angleBuffer.get(i);
            count++;
        }

        finalAngle = sum / count;
        updateServoToAngle(finalAngle);
    }

    private void updateServoToAngle(double angle) {
        clawRollServoPos = (angle + 90) / 180.0;
        clawRollServoPos = Math.max(0.0, Math.min(1.0, clawRollServoPos));
        clawRollServo.setPosition(clawRollServoPos);
    }

    private void handleServoControls(double deltaTime) {
        if (!isCollectingData && finalAngle == null) {
            handleRollServo(deltaTime);
        }

        handlePitchServo(deltaTime);

        handleClawServo();
    }

    private void handleRollServo(double deltaTime) {
        double joystickInput = applyDeadzone(-gamepad1.right_stick_y);
        double deltaPosition = joystickInput * MAX_SERVO_SPEED * deltaTime;
        clawRollServoPos = constrainServoPosition(clawRollServoPos + deltaPosition);
        clawRollServo.setPosition(clawRollServoPos);
    }

    private void handlePitchServo(double deltaTime) {
        double joystickInput = applyDeadzone(-gamepad2.left_stick_y);
        double deltaPosition = joystickInput * MAX_SERVO_SPEED * deltaTime;
        clawPitchServoPos = constrainServoPosition(clawPitchServoPos + deltaPosition);
        clawPitchServo.setPosition(clawPitchServoPos);
    }

    private void handleClawServo() {
        if (gamepad1.a && !previousAState) {
            isClawOpen = !isClawOpen;
            clawServo.setPosition(isClawOpen ? CLAW_MAX_POSITION : CLAW_MIN_POSITION);
        }
        previousAState = gamepad1.a;
    }

    private double applyDeadzone(double input) {
        if (Math.abs(input) < DEADZONE) {
            return 0;
        }
        return (Math.abs(input) - DEADZONE) / (1.0 - DEADZONE) * Math.signum(input);
    }

    private double constrainServoPosition(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }

    private void updateTelemetry() {
        telemetry.addLine("\nControls");
        telemetry.addData("Collection Status", isCollectingData ? "COLLECTING" :
                (finalAngle != null ? "FINAL POSITION" : "MANUAL"));
        telemetry.addData("Start Collection", "D-Pad Up");
        telemetry.addData("Stop and Set Position", "D-Pad Down");
        telemetry.addData("Open/Close", "Press A");

        telemetry.addLine("\nServo Positions");
        telemetry.addData("Roll", "%.3f", clawRollServoPos);
        telemetry.addData("Pitch", "%.3f", clawPitchServoPos);
        telemetry.addData("Claw", isClawOpen ? "OPEN" : "CLOSED");

        telemetry.addLine("\nBlue Object Detection");
        telemetry.addData("Detected", pipeline.isBlueObjectDetected() ? "YES" : "NO");

        if (isCollectingData) {
            telemetry.addData("Samples Collected", angleBuffer.size());
        }
        if (finalAngle != null) {
            telemetry.addData("Final Angle", String.format(Locale.US, "%.1f°", finalAngle));
        }

        if (pipeline.isBlueObjectDetected()) {
            Point center = pipeline.getObjectCenter();
            telemetry.addData("Center", String.format(Locale.US, "x=%.0f, y=%.0f",
                    center.x, center.y));
            telemetry.addData("Size", String.format(Locale.US, "%.0f pixels",
                    pipeline.getLargestContourArea()));
            telemetry.addData("Current Angle", String.format(Locale.US, "%.1f°",
                    pipeline.getAngle()));
            telemetry.addData("Rectangle", String.format(Locale.US, "%.0fx%.0f",
                    pipeline.getRectangleWidth(), pipeline.getRectangleHeight()));
        }

        telemetry.update();
    }

    private void cleanup() {
        if (camera != null) {
            camera.stopStreaming();
        }
        if (pipeline != null) {
            pipeline.release();
        }
    }
}