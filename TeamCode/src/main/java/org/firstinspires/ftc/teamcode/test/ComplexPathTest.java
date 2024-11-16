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
//import org.firstinspires.ftc.teamcode.geometry.Vector2d;
//import org.firstinspires.ftc.teamcode.trajectories.Trajectory;
//import org.firstinspires.ftc.teamcode.trajectories.TrajectoryBuilder;
//
//@Config
//@Autonomous(group = "test")
//public class ComplexPathTest extends LinearOpMode {
//    public static double RADIUS = 500;
//    public static int NUM_CYCLES = 2;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        PinpointDrive drive = new PinpointDrive(hardwareMap, telemetry);
//        ElapsedTime timer = new ElapsedTime();
//
//        // Menu for selecting test pattern
//        int selectedPattern = 0;
//        String[] patterns = {"Figure 8", "Diamond", "Circle", "S-Curve"};
//
//        while (!isStarted() && !isStopRequested()) {
//            if (gamepad1.dpad_up) {
//                selectedPattern = (selectedPattern + 1) % patterns.length;
//                sleep(200);
//            }
//            if (gamepad1.dpad_down) {
//                selectedPattern = (selectedPattern - 1 + patterns.length) % patterns.length;
//                sleep(200);
//            }
//            telemetry.addData("Selected Pattern", patterns[selectedPattern]);
//            telemetry.update();
//        }
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        drive.setPoseEstimate(new Pose2d(0, 0, 0));
//        timer.reset();
//
//        for (int cycle = 0; cycle < NUM_CYCLES; cycle++) {
//            switch(selectedPattern) {
//                case 0: // Figure 8
//                    Trajectory figure8 = drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(RADIUS, RADIUS, Math.PI/2))
//                            .splineTo(new Pose2d(0, RADIUS*2, Math.PI))
//                            .splineTo(new Pose2d(-RADIUS, RADIUS, -Math.PI/2))
//                            .splineTo(new Pose2d(0, 0, 0))
//                            .build();
//                    drive.followTrajectory(figure8);
//                    break;
//
//                case 1: // Diamond
//                    Trajectory diamond = drive.trajectoryBuilder()
//                            .lineTo(new Vector2d(RADIUS, RADIUS))
//                            .lineTo(new Vector2d(0, RADIUS*2))
//                            .lineTo(new Vector2d(-RADIUS, RADIUS))
//                            .lineTo(new Vector2d(0, 0))
//                            .build();
//                    drive.followTrajectory(diamond);
//                    break;
//
//                case 2: // Circle
//                    TrajectoryBuilder circleBuilder = drive.trajectoryBuilder();
//                    int steps = 8;
//                    for (int i = 0; i <= steps; i++) {
//                        double angle = 2 * Math.PI * i / steps;
//                        double nextAngle = angle + Math.PI/2;
//                        circleBuilder.splineTo(new Pose2d(
//                                RADIUS * Math.cos(angle),
//                                RADIUS * Math.sin(angle),
//                                nextAngle
//                        ));
//                    }
//                    drive.followTrajectory(circleBuilder.build());
//                    break;
//
//                case 3: // S-Curve
//                    Trajectory sCurve1 = drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(RADIUS/2, RADIUS, Math.PI/2))
//                            .splineTo(new Pose2d(RADIUS, RADIUS*2, 0))
//                            .build();
//                    drive.followTrajectory(sCurve1);
//
//                    Trajectory sCurve2 = drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(RADIUS/2, RADIUS, -Math.PI/2))
//                            .splineTo(new Pose2d(0, 0, Math.PI))
//                            .build();
//                    drive.followTrajectory(sCurve2);
//                    break;
//            }
//
//            telemetry.addData("Pattern", patterns[selectedPattern]);
//            telemetry.addData("Cycle", cycle + 1);
//            telemetry.addData("Runtime", timer.seconds());
//            Pose2d pose = drive.getPoseEstimate();
//            telemetry.addData("X", pose.position.x);
//            telemetry.addData("Y", pose.position.y);
//            telemetry.addData("Heading", Math.toDegrees(pose.heading));
//            telemetry.update();
//        }
//
//        while (opModeIsActive()) {
//            Pose2d finalPose = drive.getPoseEstimate();
//            telemetry.addData("Final X", finalPose.position.x);
//            telemetry.addData("Final Y", finalPose.position.y);
//            telemetry.addData("Final Heading", Math.toDegrees(finalPose.heading));
//            telemetry.addData("Total Runtime", timer.seconds());
//            telemetry.update();
//        }
//    }
//}