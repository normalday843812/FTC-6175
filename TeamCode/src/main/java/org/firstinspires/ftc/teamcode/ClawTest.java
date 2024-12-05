package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Monkey", group="TeleOp")
public class ClawTest extends LinearOpMode {
    private OpenCvCamera camera;
    private SampleDetectionPipeline pipeline;
    private static final double CLAW_MIN_POSITION = 0.14;
    private static final double CLAW_MAX_POSITION = 0.39;
    private static final double DEADZONE = 0.1;
    private static final double MAX_SERVO_SPEED = 1.0;
    HardwareMapThing robot = new HardwareMapThing();

    // State variables
    private boolean isClawOpen = false;
    private boolean previousAState = false;
    private boolean previousYState = false;  // For tracking toggle
    private boolean isTrackingEnabled = false;
    private double clawRollServoPos = 0.5;
    private double clawPitchServoPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeCamera();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        long lastUpdateTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
            lastUpdateTime = currentTime;

            handleControls(deltaTime);
            updateTelemetry();
        }

        cleanup();
    }
    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        pipeline = new SampleDetectionPipeline(SampleDetectionPipeline.ColorPair.YELLOW_BLUE);
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

    private void handleControls(double deltaTime) {
        // Toggle tracking with Y button
        if (gamepad1.y && !previousYState) {
            isTrackingEnabled = !isTrackingEnabled;
        }
        previousYState = gamepad1.y;

        // Handle tracking if enabled
        if (isTrackingEnabled && pipeline.isSampleDetected()) {
            if (!pipeline.isLocked()) {
                // Only update position if not locked onto a target
                updateServoToAngle(pipeline.getAngle());
            }
        } else {
            // Manual controls when not tracking or no sample detected
            handleManualControls(deltaTime);
        }

        // Claw control is always available
        handleClawServo();
    }

    private void handleManualControls(double deltaTime) {
        // Roll control (only if not tracking)
        double rollInput = applyDeadzone(-gamepad1.right_stick_y);
        double rollDelta = rollInput * MAX_SERVO_SPEED * deltaTime;
        clawRollServoPos = constrainServoPosition(clawRollServoPos + rollDelta);
        robot.clawRollServo.setPosition(clawRollServoPos);

        // Pitch control
        double pitchInput = applyDeadzone(-gamepad2.left_stick_y);
        double pitchDelta = pitchInput * MAX_SERVO_SPEED * deltaTime;
        clawPitchServoPos = constrainServoPosition(clawPitchServoPos + pitchDelta);
        robot.clawPitchServo.setPosition(clawPitchServoPos);
    }

    private void updateServoToAngle(double angle) {
        clawRollServoPos = (angle + 90) / 180.0;
        clawRollServoPos = constrainServoPosition(clawRollServoPos);
        robot.clawRollServo.setPosition(clawRollServoPos);
    }

    private void handleClawServo() {
        if (gamepad1.a && !previousAState) {
            isClawOpen = !isClawOpen;
            robot.clawServo.setPosition(isClawOpen ? CLAW_MAX_POSITION : CLAW_MIN_POSITION);
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

    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        telemetry.addData("Mode", isTrackingEnabled ? "TRACKING" : "MANUAL");
        telemetry.addData("Sample Detected", pipeline.isSampleDetected());

        if (pipeline.isSampleDetected()) {
            telemetry.addData("Color", pipeline.getColor());
            telemetry.addData("Confidence", String.format("%.2f", pipeline.getConfidence()));
            telemetry.addData("Angle", String.format("%.1f°", pipeline.getAngle()));
            telemetry.addData("Locked", pipeline.isLocked());
            telemetry.addData("Area", String.format("%.0f", pipeline.getArea()));
        }

        telemetry.addLine("\nServo Positions");
        telemetry.addData("Roll", String.format("%.3f", clawRollServoPos));
        telemetry.addData("Pitch", String.format("%.3f", clawPitchServoPos));
        telemetry.addData("Claw", isClawOpen ? "OPEN" : "CLOSED");

        telemetry.addLine("\nControls");
        telemetry.addData("Y Button", "Toggle Tracking");
        telemetry.addData("A Button", "Toggle Claw");
        telemetry.addData("Right Stick", "Roll (Manual)");
        telemetry.addData("Left Stick (GP2)", "Pitch");

        telemetry.update();
    }

    private void cleanup() {
        if (camera != null) {
            camera.stopStreaming();
        }
    }
}