//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.PinpointDrive;
//import org.firstinspires.ftc.teamcode.geometry.Pose2d;
//import org.firstinspires.ftc.teamcode.trajectories.Trajectory;
//
//@Config
//@Autonomous(group = "tuning")
//public class FieldCoordinateTest extends LinearOpMode {
//    // Field coordinates in mm
//    public static double RED_NET_ZONE_X = -1400;
//    public static double RED_NET_ZONE_Y = -1400;
//
//    public static double ASCENT_ZONE_X = 0;
//    public static double ASCENT_ZONE_Y = 0;
//
//    public static double SUBMERSIBLE_X = -700;
//    public static double SUBMERSIBLE_Y = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        PinpointDrive drive = new PinpointDrive(hardwareMap, telemetry);
//
//        // Menu for selecting test points
//        boolean testing = true;
//        int selectedPoint = 0;
//        String[] testPoints = {"Red Net Zone", "Ascent Zone", "Submersible"};
//
//        while (!isStarted() && !isStopRequested()) {
//            if (gamepad1.dpad_up) {
//                selectedPoint = (selectedPoint + 1) % testPoints.length;
//                sleep(200);
//            }
//            if (gamepad1.dpad_down) {
//                selectedPoint = (selectedPoint - 1 + testPoints.length) % testPoints.length;
//                sleep(200);
//            }
//
//            telemetry.addData("Press dpad up/down to select test point", "");
//            telemetry.addData("Selected Point", testPoints[selectedPoint]);
//            telemetry.update();
//        }
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && testing) {
//            drive.setPoseEstimate(new Pose2d(0, 0, 0));
//
//            Pose2d targetPose;
//            switch(selectedPoint) {
//                case 0:
//                    targetPose = new Pose2d(RED_NET_ZONE_X, RED_NET_ZONE_Y, Math.PI/2);
//                    break;
//                case 1:
//                    targetPose = new Pose2d(ASCENT_ZONE_X, ASCENT_ZONE_Y, Math.PI/2);
//                    break;
//                case 2:
//                    targetPose = new Pose2d(SUBMERSIBLE_X, SUBMERSIBLE_Y, 0);
//                    break;
//                default:
//                    targetPose = new Pose2d(0, 0, 0);
//            }
//
//            Trajectory trajectory = drive.trajectoryBuilder()
//                    .splineTo(targetPose)
//                    .build();
//
//            drive.followTrajectory(trajectory);
//
//            // Show position error
//            Pose2d finalPose = drive.getPoseEstimate();
//            double positionError = Math.hypot(
//                    finalPose.position.x - targetPose.position.x,
//                    finalPose.position.y - targetPose.position.y
//            );
//            double headingError = Math.abs(finalPose.heading - targetPose.heading);
//
//            telemetry.addData("Test Point", testPoints[selectedPoint]);
//            telemetry.addData("Position Error (mm)", positionError);
//            telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
//            telemetry.addData("Press X to test again, B to select new point, Y to end", "");
//            telemetry.update();
//
//            // Wait for input
//            while (opModeIsActive() && !gamepad1.x && !gamepad1.b && !gamepad1.y) {
//                sleep(20);
//            }
//            if (gamepad1.b) {
//                // Return to point selection
//                while (opModeIsActive() && gamepad1.b) sleep(20);
//            } else if (gamepad1.y) {
//                testing = false;
//            }
//            sleep(200);
//        }
//    }
//}