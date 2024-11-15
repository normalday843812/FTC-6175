package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectories.Trajectory;

@Config
@Autonomous(group = "tuning")
public class SplineTest extends LinearOpMode {
    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING = 0;

    public static double END_X = 1000;
    public static double END_Y = 1000;
    public static double END_HEADING = Math.PI/2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PinpointDrive drive = new PinpointDrive(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(new Pose2d(START_X, START_Y, START_HEADING));

            Trajectory trajectory = drive.trajectoryBuilder()
                    .splineTo(new Pose2d(END_X, END_Y, END_HEADING))
                    .build();

            drive.followTrajectory(trajectory);

            telemetry.addData("Current X", drive.getPoseEstimate().position.x);
            telemetry.addData("Current Y", drive.getPoseEstimate().position.y);
            telemetry.addData("Current Heading", Math.toDegrees(drive.getPoseEstimate().heading));
            telemetry.update();

            sleep(2000);
        }
    }
}