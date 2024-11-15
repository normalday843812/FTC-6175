package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;

@Config
@Autonomous(group = "test")
public class PinpointCalibrationTest extends LinearOpMode {
    public static double X_OFFSET = -84.0; // mm, default from documentation
    public static double Y_OFFSET = -168.0; // mm, default from documentation
    public static double TEST_DISTANCE = 500; // mm

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PinpointDrive drive = new PinpointDrive(hardwareMap, telemetry);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("X Offset (mm)", X_OFFSET);
            telemetry.addData("Y Offset (mm)", Y_OFFSET);
            telemetry.addData("Instructions", "Adjust offsets in Dashboard");
            telemetry.addData("Test Steps", "1. Forward/Back\n2. Left/Right\n3. Rotation");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Test forward/backward
        telemetry.addData("Test", "Forward/Back");
        telemetry.update();

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(TEST_DISTANCE, 0))
                .build());
        sleep(1000);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0, 0))
                .build());
        sleep(1000);

        // Test left/right
        telemetry.addData("Test", "Left/Right");
        telemetry.update();

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0, TEST_DISTANCE))
                .build());
        sleep(1000);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0, 0))
                .build());
        sleep(1000);

        // Test rotation
        telemetry.addData("Test", "Rotation");
        telemetry.update();

        for (int i = 0; i < 4; i++) {
            drive.turnTo(new Vector2d(Math.cos(Math.PI/2), Math.sin(Math.PI/2)));
            sleep(1000);
        }

        while (opModeIsActive()) {
            Pose2d pose = drive.getPoseEstimate();
            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            telemetry.addData("Heading", Math.toDegrees(pose.heading));
            telemetry.addData("Status", "Complete - Check return accuracy");
            telemetry.update();
        }
    }
}

