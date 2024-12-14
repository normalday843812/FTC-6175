package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.*;
@Config
@TeleOp(name="Final Teleop For Competition", group="Teleop")
public class FullThing extends LinearOpMode {
    HardwareMapThing robot = new HardwareMapThing();
    private boolean isClawOpen = false;
    private boolean previousAState = false;
    private boolean previousYState = false;
    private boolean previousBState = false;
    private boolean previousXState = false;
    private boolean previousDpadRightState = false;

    // Servo positions and initial values
    private double clawRollServoPos = 0.5;
    private double clawPitchServoPos = 0.5;
    private double armServoPosition = 0.5;
    private double armPitchServoPosition = 0.5;
    private boolean slowMode = false;
    private boolean left_bumper_previous = false;

    // PIDF parameters for bucket motors, tunable via FTC Dashboard
    public static double p = 50.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;

    // Target position for the bucket lift
    private int bucketTargetPosition = 0;

    // Convert old per-loop increments to per-second increments
    // Originally: armServo += 0.00175 per loop (approx 50 loops/sec) => 0.00175*50 = 0.0875 per sec
    private static final double ARM_EXTEND_SPEED_PER_SECOND = 0.4;

    // We'll cast the bucket motors to DcMotorEx after init
    private DcMotorEx bucketMotorEx0 = null;
    private DcMotorEx bucketMotorEx1 = null;

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
    private final Macro moveToSampleMacro = new Macro(
            new MacroStep(100, () -> {
                if (robot.clawServo != null) robot.clawServo.setPosition(HardwareMapThing.CLAW_MAX_POSITION);
                if (robot.ArmPitchServo0 != null) robot.ArmPitchServo0.setPosition(0.824);
                if (robot.ArmPitchServo1 != null) robot.ArmPitchServo1.setPosition(0.176); // 1.0 - 0.824
                if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(0.4);
                if (robot.clawRollServo != null) robot.clawRollServo.setPosition(0.5);
                isClawOpen = true;
            }),
            new MacroStep(1000, () -> {
                if (robot.ArmServo0 != null) robot.ArmServo0.setPosition(0.5);
                if (robot.ArmServo1 != null) robot.ArmServo1.setPosition(0.5);
                if (robot.clawRollServo != null) robot.clawRollServo.setPosition(0.76);
            })
    );

    // "Pick up sample" macro
    private final Macro pickupSampleMacro = new Macro(
            new MacroStep(100, () -> {
                if (robot.ArmPitchServo0 != null) robot.ArmPitchServo0.setPosition(1.0);
                if (robot.ArmPitchServo1 != null) robot.ArmPitchServo1.setPosition(0.0);
            }),
            new MacroStep(250, () -> {
                if (robot.clawServo != null) robot.clawServo.setPosition(HardwareMapThing.CLAW_MIN_POSITION);
                if (robot.ArmPitchServo0 != null) robot.ArmPitchServo0.setPosition(0.5);
                if (robot.ArmPitchServo1 != null) robot.ArmPitchServo1.setPosition(0.5);
                isClawOpen = false;
            })
    );

    private final Macro moveToSpecimenMacro = new Macro(
            new MacroStep(300, () -> {
                if (robot.ArmServo0 != null) robot.ArmServo0.setPosition(1.0);
                if (robot.ArmServo1 != null) robot.ArmServo1.setPosition(0.0);
                if (robot.ArmPitchServo0 != null) robot.ArmPitchServo0.setPosition(0.1);
                if (robot.ArmPitchServo1 != null) robot.ArmPitchServo1.setPosition(0.9);
                if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(0.0);
            }),
            new MacroStep(400, () -> {
                if (robot.clawRollServo != null) robot.clawRollServo.setPosition(0.4);
            }),
            new MacroStep(400, () -> {
                if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(0.4);
                if (robot.clawRollServo != null) robot.clawRollServo.setPosition(0.5);
                if (robot.clawServo != null) robot.clawServo.setPosition(HardwareMapThing.CLAW_MAX_POSITION);
                isClawOpen = true;
            })
    );

    private final Macro pickupSpecimenMacro = new Macro(
            new MacroStep(100, () -> {
                if (robot.clawServo != null) robot.clawServo.setPosition(HardwareMapThing.CLAW_MIN_POSITION);
                isClawOpen = false;
            }),
            new MacroStep(1000, () -> {
                if (bucketMotorEx0 != null) {
                    bucketMotorEx0.setTargetPosition(1000);
                    bucketMotorEx0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucketMotorEx0.setPower(1.0);
                }
                if (bucketMotorEx1 != null) {
                    bucketMotorEx1.setTargetPosition(1000);
                    bucketMotorEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucketMotorEx1.setPower(1.0);
                }
            }),
            new MacroStep(1000, () -> {
                if (bucketMotorEx0 != null) {
                    bucketMotorEx0.setTargetPosition(0);
                    bucketMotorEx0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucketMotorEx0.setPower(1.0);
                }
                if (bucketMotorEx1 != null) {
                    bucketMotorEx1.setTargetPosition(0);
                    bucketMotorEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucketMotorEx1.setPower(1.0);
                }
            }),
            new MacroStep(200, () -> {
                if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(0.639);
            }),
            new MacroStep(200, () -> {
                if (robot.clawRollServo != null) robot.clawRollServo.setPosition(1.0);
            }),
            new MacroStep(200, () -> {
                if (robot.ArmPitchServo0 != null) robot.ArmPitchServo0.setPosition(0.55);
                if (robot.ArmPitchServo1 != null) robot.ArmPitchServo1.setPosition(0.45);
                if (robot.ArmServo0 != null) robot.ArmServo0.setPosition(0.7);
                if (robot.ArmServo1 != null) robot.ArmServo1.setPosition(0.3);
                if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(0.381);
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

        // Cast the bucket motors to DcMotorEx, if available
        if (robot.BucketMotor0 != null && robot.BucketMotor0 instanceof DcMotorEx) {
            bucketMotorEx0 = (DcMotorEx) robot.BucketMotor0;
        }
        if (robot.BucketMotor1 != null && robot.BucketMotor1 instanceof DcMotorEx) {
            bucketMotorEx1 = (DcMotorEx) robot.BucketMotor1;
        }

        // Set initial modes for the bucket motors
        if (bucketMotorEx0 != null) {
            bucketMotorEx0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bucketMotorEx0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bucketMotorEx0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (bucketMotorEx1 != null) {
            bucketMotorEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bucketMotorEx1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bucketMotorEx1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

                    // Start macros based on button presses if no macro is running
                    // Gamepad 2 Controls for Macros:
                    // Y button: Move to sample position
                    if (gamepad2.y && !previousYState && !macroRunner.isRunning()) {
                        telemetry.addLine("Starting moveToSample macro");
                        macroRunner.startMacro(moveToSampleMacro);
                    }

                    // B button: Pick up sample
                    if (gamepad2.b && !previousBState && !macroRunner.isRunning()) {
                        telemetry.addLine("Starting pickupSample macro");
                        macroRunner.startMacro(pickupSampleMacro);
                    }

                    // X button: Move to specimen position
                    if (gamepad2.x && !previousXState && !macroRunner.isRunning()) {
                        telemetry.addLine("Starting moveToSpecimen macro");
                        macroRunner.startMacro(moveToSpecimenMacro);
                    }

                    // DPAD right: Pick up specimen
                    if (gamepad2.dpad_right && !previousDpadRightState && !macroRunner.isRunning()) {
                        telemetry.addLine("Starting pickupSpecimen macro");
                        macroRunner.startMacro(pickupSpecimenMacro);
                    }
                }

                // Update all button states
                previousYState = gamepad2.y;
                previousBState = gamepad2.b;
                previousXState = gamepad2.x;
                previousDpadRightState = gamepad2.dpad_right;

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

        double y = gamepad1.left_stick_y * (slowMode ? 0.3 : 1);
        double x = gamepad1.left_stick_x * (slowMode ? 0.3 : 1) * 1.1;
        double rx = gamepad1.right_stick_x * (slowMode ? 0.3 : 1);

        if (gamepad1.left_bumper && !left_bumper_previous) {
            slowMode = !slowMode;
        }
        left_bumper_previous = gamepad1.left_bumper;

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

        double pitchInput = applyDeadzone(-gamepad2.right_stick_x);
        double pitchDelta = pitchInput * HardwareMapThing.MAX_SERVO_SPEED * deltaTime;
        clawPitchServoPos = constrainServoPosition(clawPitchServoPos + pitchDelta);
        if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(clawPitchServoPos);

        double armInputHigher = gamepad2.right_trigger;
        double armInputLower = gamepad2.left_trigger;

        double armPitchInputHigher = gamepad2.right_bumper ? 1 : 0;
        double armPitchInputLower = gamepad2.left_bumper ? 1 : 0;

        // Now scale arm movements by deltaTime for consistent speed
        armServoPosition += (armInputHigher - armInputLower) * ARM_EXTEND_SPEED_PER_SECOND * deltaTime;
        armServoPosition = Math.max(0.0, Math.min(1.0, armServoPosition));

        armPitchServoPosition += (armPitchInputHigher - armPitchInputLower) * ARM_EXTEND_SPEED_PER_SECOND * deltaTime;
        armPitchServoPosition = Math.max(0.0, Math.min(1.0, armPitchServoPosition));

        applyArmPositions();

        // Adjust the target position of the bucket
        if (gamepad2.dpad_up) {
            bucketTargetPosition = -10;
        } else if (gamepad2.dpad_down) {
            bucketTargetPosition = 10;
        } else {
            bucketTargetPosition = 0;
        }

        // Update PIDF coefficients from dashboard values and apply to motors
        if (bucketMotorEx0 != null && bucketMotorEx1 != null) {
            PIDFCoefficients velocityPIDF = new PIDFCoefficients(p, i, d, f);
            bucketMotorEx0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
            bucketMotorEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);

            // Set position P for RUN_TO_POSITION
            bucketMotorEx0.setPositionPIDFCoefficients(p);
            bucketMotorEx1.setPositionPIDFCoefficients(p);

            // Move the bucket to the target using RUN_TO_POSITION
            bucketMotorEx0.setTargetPosition(bucketTargetPosition);
            bucketMotorEx1.setTargetPosition(bucketTargetPosition);

            bucketMotorEx0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bucketMotorEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            bucketMotorEx0.setPower(1.0);
            bucketMotorEx1.setPower(1.0);
        } else {
            telemetry.addLine("Bucket motors not configured as DcMotorEx!");
        }
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
        telemetry.addData("bucket target", bucketTargetPosition);
        if (robot.BucketMotor0 != null) telemetry.addData("bucket pos", robot.BucketMotor0.getCurrentPosition());
        if (robot.BucketMotor1 != null) telemetry.addData("bucket pos2", robot.BucketMotor1.getCurrentPosition());
        if (robot.BucketMotor0 != null) telemetry.addData("Target pos", robot.BucketMotor0.getTargetPosition());
        if (robot.BucketMotor1 != null) telemetry.addData("Target pos2", robot.BucketMotor1.getTargetPosition());

        telemetry.addData("Claw", isClawOpen ? "open" : "closed");
        telemetry.addData("tRoll", "%.3f", clawRollServoPos);
        telemetry.addData("Pitch", "%.3f", clawPitchServoPos);
        telemetry.addData("SlowMode", slowMode ? "Enabled" : "Disabled");
        telemetry.addData("ArmPitch", "%.3f", armPitchServoPosition);
        telemetry.addData("Arm", "%.3f", armServoPosition);

        telemetry.addData("Macro Running", macroRunner.isRunning());
        telemetry.addData("PIDF", "p:%.3f i:%.3f d:%.3f f:%.3f", p, i, d, f);
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