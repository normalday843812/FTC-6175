package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectories.Trajectory;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;

@Config
@Autonomous(name="Auto Final", group="Competition")
public class AutonomousFinal extends LinearOpMode {
    // Configuration constants
    public static double START_X = -1400;
    public static double START_Y = -1400;
    public static double ASCENT_X = 0;
    public static double ASCENT_Y = 0;

    public static double HANG_MOTOR_SPEED = 1.0;
    public static int HANG_MOTOR_TARGET = 1000;
    public static double HANG_TIMEOUT = 5.0;

    // Error thresholds
    private static final int ENCODER_TOLERANCE = 50;
    private static final double MOTOR_STALL_THRESHOLD = 0.1;
    private static final long STALL_CHECK_INTERVAL_MS = 500;
    private static final int MAX_RETRY_ATTEMPTS = 3;

    // Class members
    private DcMotorEx hangMotorL;
    private DcMotorEx hangMotorR;
    private PinpointDrive drive;
    private ElapsedTime runtime;
    private boolean isError = false;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();

            AndroidSoundPool androidSoundPool = new AndroidSoundPool();
            androidSoundPool.initialize(SoundPlayer.getInstance());
            androidSoundPool.preloadSound("fun.mp3");

            telemetry.addLine("Initialization complete - Ready!");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;
            executeAutonomous();
            androidSoundPool.close();
        } catch (Exception e) {
            handleFatalError("Fatal error in autonomous", e);
        } finally {
            // Ensure motors are stopped regardless of how we exit
            safeShutdown();
        }
    }

    private void initialize() {
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            runtime = new ElapsedTime();

            initializeDrive();
            initializeHangMotors();
            validateConfiguration();

        } catch (Exception e) {
            handleFatalError("Initialization failed", e);
        }
    }

    private void initializeDrive() {
        try {
            drive = new PinpointDrive(hardwareMap, telemetry);
            drive.setPoseEstimate(new Pose2d(START_X, START_Y, Math.PI/2));
        } catch (Exception e) {
            handleFatalError("Drive initialization failed", e);
        }
    }

    private void initializeHangMotors() {
        try {
            hangMotorL = hardwareMap.get(DcMotorEx.class, "hangMotorLeft");
            hangMotorR = hardwareMap.get(DcMotorEx.class, "hangMotorRight");

            // Configure motors with error checking
            if (configureMotor(hangMotorL, "Left Hang Motor") ||
                    configureMotor(hangMotorR, "Right Hang Motor")) {
                throw new RuntimeException("Motor configuration failed");
            }
        } catch (Exception e) {
            handleFatalError("Hang motor initialization failed", e);
        }
    }

    private boolean configureMotor(DcMotorEx motor, String motorName) {
        try {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thread.sleep(50); // Give time for encoder reset

            if (Math.abs(motor.getCurrentPosition()) > 10) {
                telemetry.addData("Error", motorName + " encoder did not reset properly");
                telemetry.update();
                return true;
            }

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return false;
        } catch (Exception e) {
            telemetry.addData("Error", motorName + " configuration failed: " + e.getMessage());
            telemetry.update();
            return true;
        }
    }

    private void validateConfiguration() {
        // Validate static configuration parameters
        if (HANG_MOTOR_SPEED <= 0 || HANG_MOTOR_SPEED > 1.0) {
            throw new IllegalArgumentException("Invalid HANG_MOTOR_SPEED: " + HANG_MOTOR_SPEED);
        }
        if (HANG_MOTOR_TARGET <= 0) {
            throw new IllegalArgumentException("Invalid HANG_MOTOR_TARGET: " + HANG_MOTOR_TARGET);
        }
        if (HANG_TIMEOUT <= 0) {
            throw new IllegalArgumentException("Invalid HANG_TIMEOUT: " + HANG_TIMEOUT);
        }
    }

    private void executeAutonomous() {
        try {
            // Execute trajectory with retry logic
            executeTrajectoryWithRetry();

            if (!isError) {
                // Execute hanging sequence
                executeHangSequence();

                if (!isError) {
                    executePullUp();
                }
            }
        } catch (Exception e) {
            handleFatalError("Autonomous execution failed", e);
        }
    }

    private void executeTrajectoryWithRetry() {
        Trajectory toAscentZone = drive.trajectoryBuilder()
                .splineTo(new Pose2d(ASCENT_X, ASCENT_Y, Math.PI/2))
                .build();

        for (int attempt = 1; attempt <= MAX_RETRY_ATTEMPTS; attempt++) {
            try {
                telemetry.addData("Trajectory Attempt", attempt);
                telemetry.update();

                drive.followTrajectory(toAscentZone);
                return; // Success
            } catch (Exception e) {
                if (attempt == MAX_RETRY_ATTEMPTS) {
                    handleFatalError("Failed to execute trajectory after " + MAX_RETRY_ATTEMPTS + " attempts", e);
                } else {
                    telemetry.addData("Warning", "Trajectory failed, retrying: " + e.getMessage());
                    telemetry.update();
                    sleep(1000);
                }
            }
        }
    }

    private void executeHangSequence() {
        runtime.reset();
        int previousLeftPos = 0;
        int previousRightPos = 0;
        long lastStallCheck = 0;

        try {
            // Set target positions
            hangMotorL.setTargetPosition(HANG_MOTOR_TARGET);
            hangMotorR.setTargetPosition(-HANG_MOTOR_TARGET);
            hangMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hangMotorL.setPower(HANG_MOTOR_SPEED);
            hangMotorR.setPower(HANG_MOTOR_SPEED);

            while (opModeIsActive() &&
                    runtime.seconds() < HANG_TIMEOUT &&
                    (hangMotorL.isBusy() || hangMotorR.isBusy())) {

                // Check for motor stall
                if (System.currentTimeMillis() - lastStallCheck > STALL_CHECK_INTERVAL_MS) {
                    int currentLeftPos = hangMotorL.getCurrentPosition();
                    int currentRightPos = hangMotorR.getCurrentPosition();

                    double leftProgress = Math.abs(currentLeftPos - previousLeftPos);
                    double rightProgress = Math.abs(currentRightPos - previousRightPos);

                    if (leftProgress < MOTOR_STALL_THRESHOLD || rightProgress < MOTOR_STALL_THRESHOLD) {
                        telemetry.addData("Warning", "Possible motor stall detected");
                        telemetry.update();
                    }

                    previousLeftPos = currentLeftPos;
                    previousRightPos = currentRightPos;
                    lastStallCheck = System.currentTimeMillis();
                }

                // Check for significant position mismatch
                if (Math.abs(Math.abs(hangMotorL.getCurrentPosition()) -
                        Math.abs(hangMotorR.getCurrentPosition())) > ENCODER_TOLERANCE) {
                    handleFatalError("Motor position mismatch detected", null);
                    return;
                }

                telemetry.addData("Left Pos", hangMotorL.getCurrentPosition());
                telemetry.addData("Right Pos", hangMotorR.getCurrentPosition());
                telemetry.addData("Runtime", runtime.seconds());
                telemetry.update();
            }

            // Verify final positions
            verifyMotorPositions();

        } catch (Exception e) {
            handleFatalError("Hang sequence failed", e);
        }
    }

    private void verifyMotorPositions() {
        int leftError = Math.abs(hangMotorL.getCurrentPosition() - HANG_MOTOR_TARGET);
        int rightError = Math.abs(hangMotorR.getCurrentPosition() + HANG_MOTOR_TARGET);

        if (leftError > ENCODER_TOLERANCE || rightError > ENCODER_TOLERANCE) {
            telemetry.addData("Warning", "Motor position targets not reached");
            telemetry.addData("Left Error", leftError);
            telemetry.addData("Right Error", rightError);
            telemetry.update();
        }
    }

    private void executePullUp() {
        try {
            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            sleep(500);
            drive.stopMotors();

            hangMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive()) {
                hangMotorL.setPower(-HANG_MOTOR_SPEED);
                hangMotorR.setPower(HANG_MOTOR_SPEED);

                // Monitor motor current if available
                monitorMotorHealth();

                telemetry.addData("Status", "Pulling up");
                telemetry.addData("Left Motor", hangMotorL.getCurrentPosition());
                telemetry.addData("Right Motor", hangMotorR.getCurrentPosition());
                telemetry.update();
            }
        } catch (Exception e) {
            handleFatalError("Pull up sequence failed", e);
        }
    }

    private void monitorMotorHealth() {
        try {
            // Check for significant position discrepancy
            if (Math.abs(Math.abs(hangMotorL.getCurrentPosition()) -
                    Math.abs(hangMotorR.getCurrentPosition())) > ENCODER_TOLERANCE) {
                telemetry.addData("Warning", "Motor position mismatch during pull up");
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("Warning", "Failed to monitor motor health: " + e.getMessage());
            telemetry.update();
        }
    }

    private void handleFatalError(String message, Exception e) {
        isError = true;
        telemetry.addData("ERROR", message);
        if (e != null) {
            telemetry.addData("Exception", e.getMessage());
        }
        telemetry.update();
        safeShutdown();
    }

    private void safeShutdown() {
        try {
            // Safely stop all motors
            if (hangMotorL != null) hangMotorL.setPower(0);
            if (hangMotorR != null) hangMotorR.setPower(0);
            if (drive != null) drive.stopMotors();

            telemetry.addData("Status", "Shut down complete");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to shut down safely: " + e.getMessage());
            telemetry.update();
        }
    }
}