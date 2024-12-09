package org.firstinspires.ftc.teamcode.teleopbase;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.*;

@TeleOp(name="Final Teleop For Competition", group="RUN?")
public class FullThing extends LinearOpMode {
    HardwareMapThing robot = new HardwareMapThing();
    private boolean isClawOpen = false;
    private boolean previousAState = false;
    private boolean previousYState = false;
    private boolean previousBState = false;
    private boolean previousXState = false;

    // Servo positions and initial values
    private double clawRollServoPos = 0.5;
    private double clawPitchServoPos = 0.5;
    private double armServoPosition = 0.5;
    private double armPitchServoPosition = 0.5;

    // Deadzone and servo speed constants come from HardwareMapThing
    // Assuming they are static fields in HardwareMapThing

    // --- Macro System Classes ---
    private static class MacroStep {
        long durationMs;
        Runnable action;
        MacroStep(long durationMs, Runnable action) {
            this.durationMs = durationMs;
            this.action = action;
        }
    }

    private static class Macro {
        List<MacroStep> steps;
        Macro(MacroStep... steps) {
            this.steps = Arrays.asList(steps);
        }
    }

    private static class MacroRunner {
        private Macro currentMacro = null;
        private int currentStepIndex = 0;
        private long stepStartTime = 0;
        private boolean running = false;

        void startMacro(Macro macro) {
            currentMacro = macro;
            currentStepIndex = 0;
            running = true;
            stepStartTime = System.currentTimeMillis();
            // Run first step action immediately
            currentMacro.steps.get(0).action.run();
        }

        void update() {
            if (!running || currentMacro == null) return;
            long currentTime = System.currentTimeMillis();
            MacroStep step = currentMacro.steps.get(currentStepIndex);

            if (currentTime - stepStartTime >= step.durationMs) {
                currentStepIndex++;
                if (currentStepIndex < currentMacro.steps.size()) {
                    stepStartTime = currentTime;
                    currentMacro.steps.get(currentStepIndex).action.run();
                } else {
                    // Done with macro
                    running = false;
                    currentMacro = null;
                    currentStepIndex = 0;
                }
            }
        }

        boolean isRunning() {
            return running;
        }
    }

    // --- Macros Definition ---

    // "Go to sample" macro
    // Steps:
    // 1. (100 ms): Open claw and set roll = 0.5
    // 2. (1 s): Set pitch = 1.0, armPitch = 0.824, arm = 0.387
    private final Macro moveToSampleMacro = new Macro(
            new MacroStep(100, () -> {
                // Step 1: Open claw and set roll
                isClawOpen = true;
                if (robot.clawServo != null) {
                    robot.clawServo.setPosition(HardwareMapThing.CLAW_MAX_POSITION);
                }

                // Set roll to 0.5
                clawRollServoPos = 0.5;
                if (robot.clawRollServo != null) robot.clawRollServo.setPosition(clawRollServoPos);
            }),
            new MacroStep(1000, () -> {
                // Step 2: Set pitch, arm pitch, and arm
                clawPitchServoPos = 1.0;
                armPitchServoPosition = 0.824;
                armServoPosition = 0.387;

                applyArmPositions();
                applyClawPositions();
            })
    );

    // "Pick up sample" macro
    // Steps:
    // 1. (250 ms): Close claw
    // 2. (500 ms): armPitch = 0.231, arm = 0.0, roll = 0.5, pitch = 0.588
    private final Macro pickupSampleMacro = new Macro(
            new MacroStep(250, () -> {
                // Step 1: Close claw
                isClawOpen = false;
                if (robot.clawServo != null) {
                    robot.clawServo.setPosition(HardwareMapThing.CLAW_MIN_POSITION);
                }
            }),
            new MacroStep(500, () -> {
                // Step 2: Set arm pitch, arm, roll, and pitch
                armPitchServoPosition = 0.231;
                armServoPosition = 0.0;
                clawRollServoPos = 0.5;
                clawPitchServoPos = 0.588;

                applyArmPositions();
                applyClawPositions();
            })
    );

    // "Score sample" macro
    // Steps:
    // 1. (250 ms): Open claw
    private final Macro scoreSampleMacro = new Macro(
            new MacroStep(250, () -> {
                // Step 1: Open claw
                isClawOpen = true;
                if (robot.clawServo != null) {
                    robot.clawServo.setPosition(HardwareMapThing.CLAW_MAX_POSITION);
                }
            })
    );

    private final MacroRunner macroRunner = new MacroRunner();

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine("Exception during init: " + e.getMessage());
        }

        robot.reportStatusToTelemetry(telemetry);
        telemetry.update();

        waitForStart();

        // Set initial servo positions if no errors
        if (!robot.hasHardwareError()) {
            applyArmPositions();
            applyClawPositions();
        }

        long lastUpdateTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
            lastUpdateTime = currentTime;

            robot.reportStatusToTelemetry(telemetry);

            if (!robot.hasHardwareError()) {
                // Handle manual driving
                handleDriving();

                // Only allow manual control of arm/claw if no macro is running
                if (!macroRunner.isRunning()) {
                    handleManualControls(deltaTime);

                    // Start macros based on button presses if no macro is currently running
                    if (gamepad2.y && !previousYState && !macroRunner.isRunning()) {
                        // Start "move to sample" macro
                        macroRunner.startMacro(moveToSampleMacro);
                    }
                    if (gamepad2.b && !previousBState && !macroRunner.isRunning()) {
                        // Start "pickup sample" macro
                        macroRunner.startMacro(pickupSampleMacro);
                    }
                    if (gamepad2.x && !previousXState && !macroRunner.isRunning()) {
                        // Start "score sample" macro
                        macroRunner.startMacro(scoreSampleMacro);
                    }
                }

                previousYState = gamepad2.y;
                previousBState = gamepad2.b;
                previousXState = gamepad2.x;

                // Update the macro runner
                macroRunner.update();
            } else {
                telemetry.addLine("Hardware error - not controlling robot.");
            }

            updateTelemetry();
            telemetry.update();
        }
    }

    private void handleDriving() {
        if (robot.FLMotor == null || robot.FRMotor == null || robot.BLMotor == null || robot.BRMotor == null) {
            telemetry.addLine("Cannot drive, some drive motors are missing.");
            return;
        }

        double y = gamepad1.right_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        double max = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        robot.FLMotor.setPower(frontLeftPower);
        robot.FRMotor.setPower(frontRightPower);
        robot.BLMotor.setPower(backLeftPower);
        robot.BRMotor.setPower(backRightPower);
    }

    private void handleManualControls(double deltaTime) {
        // If a macro is running, we skip manual controls
        if (macroRunner.isRunning()) {
            return;
        }

        if (gamepad2.a && !previousAState && robot.clawServo != null) {
            isClawOpen = !isClawOpen;
            robot.clawServo.setPosition(isClawOpen ? HardwareMapThing.CLAW_MAX_POSITION : HardwareMapThing.CLAW_MIN_POSITION);
        }
        previousAState = gamepad2.a;

        double rollInput = applyDeadzone(-gamepad2.right_stick_y);
        double rollDelta = rollInput * HardwareMapThing.MAX_SERVO_SPEED * deltaTime;
        clawRollServoPos = constrainServoPosition(clawRollServoPos + rollDelta);
        if (robot.clawRollServo != null) robot.clawRollServo.setPosition(clawRollServoPos);

        double pitchInput = applyDeadzone(-gamepad2.left_stick_y);
        double pitchDelta = pitchInput * HardwareMapThing.MAX_SERVO_SPEED * deltaTime;
        clawPitchServoPos = constrainServoPosition(clawPitchServoPos + pitchDelta);
        if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(clawPitchServoPos);

        double armInputHigher = gamepad2.right_trigger;
        double armInputLower = gamepad2.left_trigger;

        double armPitchInputHigher = gamepad2.right_bumper ? 1 : 0;
        double armPitchInputLower = gamepad2.left_bumper ? 1 : 0;

        armServoPosition += armInputHigher * 0.00175;
        armServoPosition -= armInputLower * 0.00175;
        armServoPosition = Math.max(0.0, Math.min(1.0, armServoPosition));

        armPitchServoPosition += armPitchInputHigher * 0.00175;
        armPitchServoPosition -= armPitchInputLower * 0.00175;
        armPitchServoPosition = Math.max(0.0, Math.min(1.0, armPitchServoPosition));

        applyArmPositions();

        double bucketInput = (gamepad2.dpad_up ? 1.0 : 0.0) - (gamepad2.dpad_down ? 1.0 : 0.0);
        if (robot.BucketMotor1 != null) robot.BucketMotor1.setPower(bucketInput); else telemetry.addLine("BucketMotor1 not found!");
        if (robot.BucketMotor0 != null) robot.BucketMotor0.setPower(bucketInput); else telemetry.addLine("BucketMotor0 not found!");
    }

    private void applyArmPositions() {
        if (robot.ArmServo0 != null) robot.ArmServo0.setPosition(armServoPosition);
        if (robot.ArmServo1 != null) robot.ArmServo1.setPosition(1.0 - armServoPosition);
        if (robot.ArmPitchServo0 != null) robot.ArmPitchServo0.setPosition(armPitchServoPosition);
        if (robot.ArmPitchServo1 != null) robot.ArmPitchServo1.setPosition(1.0 - armPitchServoPosition);
    }

    private void applyClawPositions() {
        if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(clawPitchServoPos);
        if (robot.clawRollServo != null) robot.clawRollServo.setPosition(clawRollServoPos);
    }

    private void updateTelemetry() {
        double flPower = (robot.FLMotor != null) ? robot.FLMotor.getPower() : 0;
        double frPower = (robot.FRMotor != null) ? robot.FRMotor.getPower() : 0;
        double blPower = (robot.BLMotor != null) ? robot.BLMotor.getPower() : 0;
        double brPower = (robot.BRMotor != null) ? robot.BRMotor.getPower() : 0;

        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                flPower, frPower, blPower, brPower);
        telemetry.addData("Claw", isClawOpen ? "open" : "closed");
        telemetry.addData("Roll", "%.3f", clawRollServoPos);
        telemetry.addData("Pitch", "%.3f", clawPitchServoPos);
        telemetry.addData("ArmPitch", "%.3f", armPitchServoPosition);
        telemetry.addData("Arm", "%.3f", armServoPosition);

        telemetry.addData("Macro Running", macroRunner.isRunning());
    }

    private double applyDeadzone(double input) {
        if (Math.abs(input) < HardwareMapThing.DEADZONE) {
            return 0;
        }
        return (Math.abs(input) - HardwareMapThing.DEADZONE) / (1.0 - HardwareMapThing.DEADZONE) * Math.signum(input);
    }

    private double constrainServoPosition(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }
}
