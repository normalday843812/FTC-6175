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
//public class TurnPIDTuner extends LinearOpMode {
//    public static double ANGLE = Math.PI/2; // 90 degrees
//    public static double HEADING_kP = 0;
//    public static double HEADING_kI = 0;
//    public static double HEADING_kD = 0;
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
//            // Turn right
//            drive.setPoseEstimate(new Pose2d(0, 0, 0));
//            drive.turnTo(new Vector2d(Math.cos(ANGLE), Math.sin(ANGLE)));
//            sleep(1000);
//
//            // Turn back
//            drive.turnTo(new Vector2d(1, 0));
//            sleep(1000);
//
//            telemetry.addData("Target Angle", Math.toDegrees(ANGLE));
//            telemetry.addData("Current Angle", Math.toDegrees(drive.getPoseEstimate().heading));
//            telemetry.addData("kP", HEADING_kP);
//            telemetry.addData("kI", HEADING_kI);
//            telemetry.addData("kD", HEADING_kD);
//            telemetry.update();
//        }
//    }
//}