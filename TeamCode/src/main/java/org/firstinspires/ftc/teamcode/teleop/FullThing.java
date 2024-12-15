package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.*;
@Config
@TeleOp(name="Competition Teleop", group="Teleop")
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
    // "Pick up sample" macro
    private final Macro moveToSampleMacro = new Macro(
            new MacroStep(600, () -> {
                // Step 1: Bend arm down to intake
                clawPitchServoPos = 0.391;
                clawRollServoPos = 0.944;
                armPitchServoPosition = 0.993;
                armServoPosition = 0.387;

            }),
            new MacroStep(100, () -> {
                // Step 2: Close claw
                isClawOpen = false;
                if (robot.clawServo != null) {
                    robot.clawServo.setPosition(HardwareMapThing.CLAW_MIN_POSITION);
                }
            }),
            new MacroStep(800, () -> {
                // Step 2: Set pitch, arm pitch, and arm
//              bucketTargetPosition = 0;
                clawPitchServoPos = 0.391;
                clawRollServoPos = 0.944;
                armPitchServoPosition = 0.882;
                armServoPosition = 0.387;

                applyArmPositions();
                applyClawPositions();
            })
    );

    // "Get in pos for sub intake" macro
    private final Macro pickupSampleMacro = new Macro(
            new MacroStep(100, () -> {
                // Step 1: Open claw

                isClawOpen = true;
                if (robot.clawServo != null) {
                    robot.clawServo.setPosition(HardwareMapThing.CLAW_MAX_POSITION);
                }
            }),
            new MacroStep(500, () -> {
                clawPitchServoPos = 0.391;
                clawRollServoPos = 0.944;
                armPitchServoPosition = 0.882;
                armServoPosition = 0.387;


                applyArmPositions();
                applyClawPositions();
            })
    );

    // "Get in pos to intake specimen" macro
    private final Macro scoreSampleMacro = new Macro(
            new MacroStep(100, () -> {
                // Step 1: Close claw so it can fit
                isClawOpen = false;
                if (robot.clawServo != null) {
                    robot.clawServo.setPosition(HardwareMapThing.CLAW_MIN_POSITION);
                }

            }),
            // Step 2: Set pos of all subsystems
            new MacroStep(450, () -> {
               // Set arm pitch, arm, roll, and pitch
                armPitchServoPosition = 0;
                armServoPosition = 0.931;
                clawRollServoPos = 0.552;
                clawPitchServoPos = 0.398;

//                bucketTargetPosition = 20;

                applyArmPositions();
                applyClawPositions();
            }),
            new MacroStep(100, () -> {
                // Step 3: Open Claw
                isClawOpen = true;
                    if (robot.clawServo != null) {
                        robot.clawServo.setPosition(HardwareMapThing.CLAW_MAX_POSITION);
                    }

            })
    );
     private final Macro intakeSpecimenMacro = new Macro(
             // Step 1: Close Claw
             new MacroStep(450, () -> {
                 isClawOpen = false;
                 if (robot.clawServo != null) {
                     robot.clawServo.setPosition(HardwareMapThing.CLAW_MIN_POSITION);
                 }
             }),
             new MacroStep(450, () -> {
                 // Step 2: Set arm pitch, arm, roll, and pitch
                 armPitchServoPosition = 0.576;
                 armServoPosition = 0.736;
                 clawRollServoPos = 0.767;
                 clawPitchServoPos = 0.414;

                 applyArmPositions();
                 applyClawPositions();

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

                    // Start macros based on button presses if no macro is currently running
                    if (gamepad2.y && !previousYState && !macroRunner.isRunning()) {
                        macroRunner.startMacro(moveToSampleMacro);
                    }
                    if (gamepad2.b && !previousBState && !macroRunner.isRunning()) {
                        macroRunner.startMacro(pickupSampleMacro);
                    }
                    if (gamepad2.x && !previousXState && !macroRunner.isRunning()) {
                        macroRunner.startMacro(scoreSampleMacro);
                    }
                    if  (gamepad2.a && !previousAState && !macroRunner.isRunning()) {
                        macroRunner.startMacro(intakeSpecimenMacro);
                    }
                }

                previousYState = gamepad2.y;
                previousBState = gamepad2.b;
                previousXState = gamepad2.x;
                previousAState = gamepad2.a;

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

        double y = -gamepad1.left_stick_y * (slowMode ? 1 : 0.25);
        double x = gamepad1.left_stick_x * (slowMode ? 1 : 0.25) * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x * (slowMode ? 1 : 0.25);

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

        if (gamepad1.a && !previousAState && robot.clawServo != null) {
            isClawOpen = !isClawOpen;
            robot.clawServo.setPosition(isClawOpen ? HardwareMapThing.CLAW_MAX_POSITION : HardwareMapThing.CLAW_MIN_POSITION);
        }
        previousAState = gamepad1.a;

        double rollInput = applyDeadzone(-gamepad2.right_stick_y);
        double rollDelta = rollInput * HardwareMapThing.MAX_SERVO_SPEED * deltaTime;
        clawRollServoPos = constrainServoPosition(clawRollServoPos + rollDelta);
        if (robot.clawRollServo != null) robot.clawRollServo.setPosition(clawRollServoPos);

        double pitchInput = applyDeadzone(-gamepad2.right_stick_x);
        double pitchDelta = pitchInput * HardwareMapThing.MAX_SERVO_SPEED * deltaTime;
        clawPitchServoPos = constrainServoPosition(clawPitchServoPos + pitchDelta);
        if (robot.clawPitchServo != null) robot.clawPitchServo.setPosition(clawPitchServoPos);

        double armInputHigher = Math.max(gamepad2.right_trigger+gamepad1.right_trigger,1);
        double armInputLower = Math.max(gamepad2.left_trigger+gamepad1.left_trigger,1);

        double armPitchInputHigher = gamepad2.right_bumper ? 1 : 0;
        double armPitchInputLower = gamepad2.left_bumper ? 1 : 0;

        // Now scale arm movements by deltaTime for consistent speed
        armServoPosition += (armInputHigher - armInputLower) * ARM_EXTEND_SPEED_PER_SECOND * deltaTime;
        armServoPosition = Math.max(0.0, Math.min(1.0, armServoPosition));

        armPitchServoPosition += (armPitchInputHigher - armPitchInputLower) * ARM_EXTEND_SPEED_PER_SECOND * deltaTime;
        armPitchServoPosition = Math.max(0.0, Math.min(1.0, armPitchServoPosition));

        applyArmPositions();

        // Adjust the target position of the bucket
        if (gamepad2.dpad_up || gamepad1.dpad_up) {
            bucketTargetPosition = -10;
        } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
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
        telemetry.addData("SlowMode", slowMode ? "Disabled" : "Enabled");
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
