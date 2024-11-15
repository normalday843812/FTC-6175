package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.trajectories.Trajectory;

@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointDrive drive = new PinpointDrive(hardwareMap, telemetry);

        Trajectory trajectory = drive.trajectoryBuilder()
                .lineTo(new Vector2d(DISTANCE, 0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        while (!isStopRequested() && opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading", Math.toDegrees(poseEstimate.heading));
            telemetry.update();
        }
    }
}