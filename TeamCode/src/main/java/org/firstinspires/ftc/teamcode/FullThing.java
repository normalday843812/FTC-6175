package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Full Thing", group="RUN?")
public class FullThing extends LinearOpMode {
    HardwareMapThing robot = new HardwareMapThing();
    private boolean isClawOpen = false;
    private boolean previousAState = false;
    private boolean previousYState = false;
    private double clawRollServoPos = 0.5;
    private double clawPitchServoPos = 0.5;

    // State machine variables
    private enum SequenceState {
        IDLE,
        PICKUP_OPEN_CLAW,
        PICKUP_MOVE_TO_POSITION,
        PICKUP_CLOSE_CLAW,
        PICKUP_RETURN
    }

    private SequenceState currentState = SequenceState.IDLE;
    private long stateStartTime = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        long lastUpdateTime = System.currentTimeMillis();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
            lastUpdateTime = currentTime;

            // Always handle driving and manual controls
            handleDriving();
            handleManualControls(deltaTime);

            // Check for sequence start
            if (gamepad2.y && !previousYState && currentState == SequenceState.IDLE) {
                startPickupSequence();
            }
            previousYState = gamepad2.y;

            // Handle ongoing sequence
            updateSequence(currentTime);

            updateTelemetry();
        }
    }

    private void handleDriving() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        robot.FLMotor.setPower(leftFrontPower);
        robot.FRMotor.setPower(rightFrontPower);
        robot.BLMotor.setPower(leftBackPower);
        robot.BRMotor.setPower(rightBackPower);
    }

    private void handleManualControls(double deltaTime) {
        // Only allow manual servo controls if no sequence is running
        if (currentState == SequenceState.IDLE) {
            if (gamepad2.a && !previousAState) {
                isClawOpen = !isClawOpen;
                robot.clawServo.setPosition(isClawOpen ? HardwareMapThing.CLAW_MAX_POSITION : HardwareMapThing.CLAW_MIN_POSITION);
            }
            previousAState = gamepad2.a;

            double rollInput = applyDeadzone(-gamepad2.right_stick_y);
            double rollDelta = rollInput * HardwareMapThing.MAX_SERVO_SPEED * deltaTime;
            clawRollServoPos = constrainServoPosition(clawRollServoPos + rollDelta);
            robot.clawRollServo.setPosition(clawRollServoPos);

            double pitchInput = applyDeadzone(-gamepad2.left_stick_y);
            double pitchDelta = pitchInput * HardwareMapThing.MAX_SERVO_SPEED * deltaTime;
            clawPitchServoPos = constrainServoPosition(clawPitchServoPos + pitchDelta);
            robot.clawPitchServo.setPosition(clawPitchServoPos);
        }
    }

    private void startPickupSequence() {
        currentState = SequenceState.PICKUP_OPEN_CLAW;
        stateStartTime = System.currentTimeMillis();

        // Start the sequence by opening the claw
        isClawOpen = true;
        robot.clawServo.setPosition(HardwareMapThing.CLAW_MAX_POSITION);
    }

    private void updateSequence(long currentTime) {
        if (currentState == SequenceState.IDLE) return;

        long timeInState = currentTime - stateStartTime;

        switch (currentState) {
            case PICKUP_OPEN_CLAW:
                if (timeInState >= 250) {  // After 250ms
                    // Move to position
                    robot.clawPitchServo.setPosition(0.7);
                    robot.clawRollServo.setPosition(0.3);
                    clawPitchServoPos = 0.7;
                    clawRollServoPos = 0.3;

                    currentState = SequenceState.PICKUP_MOVE_TO_POSITION;
                    stateStartTime = currentTime;
                }
                break;

            case PICKUP_MOVE_TO_POSITION:
                if (timeInState >= 500) {  // After 500ms
                    // Close claw
                    isClawOpen = false;
                    robot.clawServo.setPosition(HardwareMapThing.CLAW_MIN_POSITION);

                    currentState = SequenceState.PICKUP_CLOSE_CLAW;
                    stateStartTime = currentTime;
                }
                break;

            case PICKUP_CLOSE_CLAW:
                if (timeInState >= 250) {  // After 250ms
                    // Return to carry position
                    robot.clawPitchServo.setPosition(0.5);
                    robot.clawRollServo.setPosition(0.5);
                    clawPitchServoPos = 0.5;
                    clawRollServoPos = 0.5;

                    currentState = SequenceState.PICKUP_RETURN;
                    stateStartTime = currentTime;
                }
                break;

            case PICKUP_RETURN:
                if (timeInState >= 500) {  // After 500ms
                    // Sequence complete
                    currentState = SequenceState.IDLE;
                }
                break;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("motaksdfl", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                robot.FLMotor.getPower(), robot.FRMotor.getPower(),
                robot.BLMotor.getPower(), robot.BRMotor.getPower());
        telemetry.addData("Claw", isClawOpen ? "opne" : "closed");
        telemetry.addData("Roll", "%.3f", clawRollServoPos);
        telemetry.addData("pitch", "%.3f", clawPitchServoPos);
        telemetry.addData("Sequence State", currentState);
        telemetry.update();
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