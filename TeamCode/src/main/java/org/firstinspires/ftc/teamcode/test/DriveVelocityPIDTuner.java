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
//import org.firstinspires.ftc.teamcode.geometry.Vector2d;
//
//@Config
//@Autonomous(group = "tuning")
//public class DriveVelocityPIDTuner extends LinearOpMode {
//    public static double DISTANCE = 1000; // mm
//    public static double kP = 0;
//    public static double kI = 0;
//    public static double kD = 0;
//    public static double kV = 0.01;
//    public static double kS = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        PinpointDrive drive = new PinpointDrive(hardwareMap, telemetry);
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//            // Forward
//            drive.setPoseEstimate(new Pose2d(0, 0, 0));
//            drive.followTrajectory(
//                    drive.trajectoryBuilder()
//                            .lineTo(new Vector2d(DISTANCE, 0))
//                            .build()
//            );
//            telemetry.addData("Motor Powers", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
//                    drive.getFrontLeftPower(), drive.getFrontRightPower(),
//                    drive.getBackLeftPower(), drive.getBackRightPower());
//            sleep(1000);
//
//            // i'm gonna kms
//            drive.setPoseEstimate(new Pose2d(DISTANCE, 0, 0));
//            drive.followTrajectory(
//                    drive.trajectoryBuilder()
//                            .lineTo(new Vector2d(0, 0))
//                            .build()
//            );
//            sleep(1000);
//
//            telemetry.addData("kP", kP);
//            telemetry.addData("kI", kI);
//            telemetry.addData("kD", kD);
//            telemetry.addData("kV", kV);
//            telemetry.addData("kS", kS);
//            telemetry.update();
//        }
//    }
//}