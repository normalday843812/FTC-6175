package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.teleop.HardwareMapThing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
@Autonomous(name="2+1 Red Auto", group="Competition")
public class AutoFinal extends LinearOpMode {
    private final HardwareMapThing robot = new HardwareMapThing();

    // Starting pos
    private static final Pose2d STARTING_POSE = new Pose2d(0.25, -68.69, Math.toRadians(90.00));

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);

            MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);

            Action trajectory = drive.actionBuilder(STARTING_POSE)
                    .splineTo(new Vector2d(24.39, -39.43), Math.toRadians(3.91))
                    .splineTo(new Vector2d(39.43, 1.07), Math.toRadians(90.00))
                    .splineTo(new Vector2d(61.09, 7.19), Math.toRadians(-73.73))
                    .splineTo(new Vector2d(48.52, -35.13), Math.toRadians(270.00))
                    .splineToConstantHeading(new Vector2d(49.52, -60.26), Math.toRadians(270.00))
                    .build();





            telemetry.addData("Status", "init");
            telemetry.addData("Starting Pose", STARTING_POSE);
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {
                drive.updatePoseEstimate();

                if (!trajectory.run(new TelemetryPacket())) {
                    break;
                }

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData(" head", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();
            }

        } catch (Exception e) {
            telemetry.addData("Error", "exceptionhjre:: " + e.getMessage());
            telemetry.update();
        }
    }
}