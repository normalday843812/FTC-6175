//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.PinpointDrive;
//import org.firstinspires.ftc.teamcode.geometry.Pose2d;
//import org.firstinspires.ftc.teamcode.trajectories.Trajectory;
//
//@Config
//@Autonomous(group = "test")
//public class MotionProfileTuner extends LinearOpMode {
//    // Dashboard configurable parameters
//    public static double MAX_VEL = 1000;     // mm/s
//    public static double MAX_ACCEL = 500;    // mm/s²
//    public static double DISTANCE = 1000;    // mm
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        PinpointDrive drive = new PinpointDrive(hardwareMap, telemetry);
//        ElapsedTime timer = new ElapsedTime();
//
//        // Menu for selecting test type
//        int selectedTest = 0;
//        String[] tests = {"Velocity Profile", "Acceleration Profile", "S-Curve Profile", "Stop and Go"};
//
//        while (!isStarted() && !isStopRequested()) {
//            if (gamepad1.dpad_up) {
//                selectedTest = (selectedTest + 1) % tests.length;
//                sleep(200);
//            }
//            if (gamepad1.dpad_down) {
//                selectedTest = (selectedTest - 1 + tests.length) % tests.length;
//                sleep(200);
//            }
//            telemetry.addData("Selected Test", tests[selectedTest]);
//            telemetry.addData("MAX_VEL", MAX_VEL);
//            telemetry.addData("MAX_ACCEL", MAX_ACCEL);
//            telemetry.addData("Press Start to begin test", "");
//            telemetry.update();
//        }
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.setPoseEstimate(new Pose2d(0, 0, 0));
//            timer.reset();
//
//            switch(selectedTest) {
//                case 0: // Velocity Profile
//                    // Accelerate to max velocity
//                    Trajectory velProfile = drive.trajectoryBuilder()
//                            .forward(DISTANCE)
//                            .build();
//
//                    drive.followTrajectory(velProfile);
//                    break;
//
//                case 1: // Acceleration Profile
//                    // Quick accelerations and decelerations
//                    for (int i = 0; i < 4; i++) {
//                        Trajectory accelProfile = drive.trajectoryBuilder()
//                                .forward(DISTANCE/4)
//                                .build();
//                        drive.followTrajectory(accelProfile);
//                        sleep(500);
//                    }
//                    break;
//
//                case 2: // S-Curve Profile
//                    // Smooth acceleration and deceleration
//                    Trajectory sCurve = drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(DISTANCE/2, DISTANCE/4, Math.PI/4))
//                            .splineTo(new Pose2d(DISTANCE, 0, 0))
//                            .build();
//
//                    drive.followTrajectory(sCurve);
//                    break;
//
//                case 3: // Stop and Go
//                    // Test rapid starts and stops
//                    for (int i = 0; i < 3; i++) {
//                        Trajectory stopGo = drive.trajectoryBuilder()
//                                .forward(DISTANCE/3)
//                                .build();
//                        drive.followTrajectory(stopGo);
//                        sleep(1000);
//                    }
//                    break;
//            }
//
//            // Record and display results
//            Pose2d finalPose = drive.getPoseEstimate();
//            double actualDistance = Math.hypot(finalPose.position.x, finalPose.position.y);
//            double error = Math.abs(DISTANCE - actualDistance);
//            double averageVel = actualDistance / timer.seconds();
//
//            telemetry.addData("Test", tests[selectedTest]);
//            telemetry.addData("Target Distance", DISTANCE);
//            telemetry.addData("Actual Distance", actualDistance);
//            telemetry.addData("Error", error);
//            telemetry.addData("Average Velocity", averageVel);
//            telemetry.addData("Time", timer.seconds());
//            telemetry.addData("\nPress dpad to select new test", "");
//            telemetry.addData("Press Y to run again", "");
//            telemetry.addData("Press B to exit", "");
//            telemetry.update();
//
//            // Wait for input
//            boolean waiting = true;
//            while (opModeIsActive() && waiting) {
//                if (gamepad1.dpad_up || gamepad1.dpad_down) {
//                    break; // Select new test
//                }
//                if (gamepad1.y) {
//                    waiting = false; // Run same test again
//                }
//                if (gamepad1.b) {
//                    return; // Exit OpMode
//                }
//                sleep(20);
//            }
//        }
//    }
//}