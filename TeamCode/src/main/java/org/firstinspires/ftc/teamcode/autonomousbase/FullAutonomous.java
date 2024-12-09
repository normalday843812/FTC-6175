package org.firstinspires.ftc.teamcode.autonomousbase;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Full Autonomous INTO THE DEEP", group = "Competition")
public class FullAutonomous extends LinearOpMode {
    // Constants
    private static final double CLAW_MIN_POSITION = 0.0;
    private static final double CLAW_MAX_POSITION = 1.0;

    // Action implementations
    private static class OpenClawAction implements Action {
        private final Servo clawServo;

        OpenClawAction(Servo clawServo) {
            this.clawServo = clawServo;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(CLAW_MAX_POSITION);
            return false;
        }
    }

    private static class CloseClawAction implements Action {
        private final Servo clawServo;

        CloseClawAction(Servo clawServo) {
            this.clawServo = clawServo;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(CLAW_MIN_POSITION);
            return false;
        }
    }

    private static class SetClawOrientationAction implements Action {
        private final Servo clawPitchServo;
        private final Servo clawRollServo;
        private final double targetPitch;
        private final double targetRoll;

        SetClawOrientationAction(Servo pitchServo, Servo rollServo, double pitch, double roll) {
            this.clawPitchServo = pitchServo;
            this.clawRollServo = rollServo;
            this.targetPitch = pitch;
            this.targetRoll = roll;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawPitchServo.setPosition(targetPitch);
            clawRollServo.setPosition(targetRoll);
            return false;
        }
    }

    private static class SetArmPositionAction implements Action {
        private final Servo armServo0;
        private final Servo armServo1;
        private final Servo armPitchServo0;
        private final Servo armPitchServo1;
        private final double targetArm;
        private final double targetPitch;

        SetArmPositionAction(Servo arm0, Servo arm1, Servo pitch0, Servo pitch1,
                             double arm, double pitch) {
            this.armServo0 = arm0;
            this.armServo1 = arm1;
            this.armPitchServo0 = pitch0;
            this.armPitchServo1 = pitch1;
            this.targetArm = arm;
            this.targetPitch = pitch;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armServo0.setPosition(targetArm);
            armServo1.setPosition(1.0 - targetArm);
            armPitchServo0.setPosition(targetPitch);
            armPitchServo1.setPosition(1.0 - targetPitch);
            return false;
        }
    }

    private static class PickupSampleAction implements Action {
        private final Servo clawServo, clawPitchServo, clawRollServo;
        private final Servo armServo0, armServo1, armPitchServo0, armPitchServo1;
        private int stage = 0;
        private long startTime;

        PickupSampleAction(Servo claw, Servo clawPitch, Servo clawRoll,
                           Servo arm0, Servo arm1, Servo armPitch0, Servo armPitch1) {
            this.clawServo = claw;
            this.clawPitchServo = clawPitch;
            this.clawRollServo = clawRoll;
            this.armServo0 = arm0;
            this.armServo1 = arm1;
            this.armPitchServo0 = armPitch0;
            this.armPitchServo1 = armPitch1;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            long currentTime = System.currentTimeMillis();

            if (stage == 0) {
                startTime = currentTime;
                stage = 1;
                return true;
            }

            long elapsed = currentTime - startTime;

            if (stage == 1 && elapsed >= 0) {
                // Set initial position
                new SetClawOrientationAction(clawPitchServo, clawRollServo, 0.5, 0.5).run(packet);
                new OpenClawAction(clawServo).run(packet);
                stage = 2;
                return true;
            }

            if (stage == 2 && elapsed >= 100) {
                // Position arm for pickup
                new SetArmPositionAction(armServo0, armServo1, armPitchServo0, armPitchServo1,
                        0.387, 0.824).run(packet);
                stage = 3;
                return true;
            }

            if (stage == 3 && elapsed >= 1100) {
                // Grab sample
                new CloseClawAction(clawServo).run(packet);
                stage = 4;
                return true;
            }

            if (stage == 4 && elapsed >= 1350) {
                // Move to transport position
                new SetArmPositionAction(armServo0, armServo1, armPitchServo0, armPitchServo1,
                        0.0, 0.231).run(packet);
                new SetClawOrientationAction(clawPitchServo, clawRollServo, 0.588, 0.5).run(packet);
                stage = 5;
                return true;
            }

            return stage != 5;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        // Hardware members
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Servo clawPitchServo = hardwareMap.get(Servo.class, "clawPitchServo");
        Servo clawRollServo = hardwareMap.get(Servo.class, "clawRollServo");
        Servo armServo0 = hardwareMap.get(Servo.class, "ArmServo0");
        Servo armServo1 = hardwareMap.get(Servo.class, "ArmServo1");
        Servo armPitchServo0 = hardwareMap.get(Servo.class, "ArmPitchServo0");
        Servo armPitchServo1 = hardwareMap.get(Servo.class, "ArmPitchServo1");

        // Initialize drive system
        Pose2d startPose = new Pose2d(12.0, 62.0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build trajectory to center of field
        Action trajectoryToCenter = drive.actionBuilder(startPose)
                .splineToSplineHeading(
                        new Pose2d(36.0, 36.0, Math.toRadians(45)),
                        Math.toRadians(45)
                )
                .build();

        // Build trajectory to scoring position
        Action trajectoryToScore = drive.actionBuilder(new Pose2d(36.0, 36.0, Math.toRadians(45)))
                .splineToSplineHeading(
                        new Pose2d(48.0, 24.0, Math.toRadians(0)),
                        Math.toRadians(0)
                )
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Execute the full autonomous sequence
        Actions.runBlocking(new SequentialAction(
                trajectoryToCenter,
                new PickupSampleAction(clawServo, clawPitchServo, clawRollServo,
                        armServo0, armServo1, armPitchServo0, armPitchServo1),
                trajectoryToScore,
                new OpenClawAction(clawServo)
        ));
    }
}