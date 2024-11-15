package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;

@Config
@Autonomous(group = "test")
public class LocalizationTest extends LinearOpMode {
    public static int NUM_LAPS = 3;
    public static double SQUARE_SIZE = 500; // mm

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PinpointDrive drive = new PinpointDrive(hardwareMap, telemetry);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        timer.reset();

        for (int lap = 0; lap < NUM_LAPS; lap++) {
            // Drive in a square
            drive.followTrajectory(drive.trajectoryBuilder()
                    .lineTo(new Vector2d(SQUARE_SIZE, 0))
                    .build());

            drive.followTrajectory(drive.trajectoryBuilder()
                    .lineTo(new Vector2d(SQUARE_SIZE, SQUARE_SIZE))
                    .build());

            drive.followTrajectory(drive.trajectoryBuilder()
                    .lineTo(new Vector2d(0, SQUARE_SIZE))
                    .build());

            drive.followTrajectory(drive.trajectoryBuilder()
                    .lineTo(new Vector2d(0, 0))
                    .build());

            Pose2d pose = drive.getPoseEstimate();
            double error = Math.hypot(pose.position.x, pose.position.y);

            telemetry.addData("Lap", lap + 1);
            telemetry.addData("Runtime", timer.seconds());
            telemetry.addData("Position Error (mm)", error);
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading));
            telemetry.update();
        }

        while (opModeIsActive()) {
            Pose2d finalPose = drive.getPoseEstimate();
            telemetry.addData("Final X", finalPose.position.x);
            telemetry.addData("Final Y", finalPose.position.y);
            telemetry.addData("Final Heading", Math.toDegrees(finalPose.heading));
            telemetry.addData("Total Runtime", timer.seconds());
            telemetry.update();
        }
    }
}
