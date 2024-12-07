package org.firstinspires.ftc.teamcode.video;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Test: AprilTag Localization", group = "Test")
public class TestAprilTagLocalization extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // Initialize the AprilTag processor
        // Show axes for debugging
        // Show 3D cube for debugging
        // These are typical values, adjust if needed
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)             // Show axes for debugging
                .setDrawCubeProjection(true)   // Show 3D cube for debugging
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) // These are typical values, adjust if needed
                .build();

        // Create the vision portal
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();

        // Wait for the driver to press start
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            telemetry.addData("Number of AprilTags Detected", currentDetections.size());

            // Loop through all detections
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    // Get the robot's position relative to the tag
                    double x = detection.ftcPose.x;
                    double y = detection.ftcPose.y;
                    double z = detection.ftcPose.z;

                    // Get the robot's orientation relative to the tag
                    double yaw = detection.ftcPose.yaw;
                    double pitch = detection.ftcPose.pitch;
                    double roll = detection.ftcPose.roll;

                    telemetry.addLine(String.format("\nAprilTag ID: %d (%s)", detection.id, detection.metadata.name));
                    telemetry.addLine("==== Robot Position ====");
                    telemetry.addLine(String.format("X: %.2f inches (right/left)", x));
                    telemetry.addLine(String.format("Y: %.2f inches (forward/back)", y));
                    telemetry.addLine(String.format("Z: %.2f inches (up/down)", z));
                    telemetry.addLine("\n==== Robot Orientation ====");
                    telemetry.addLine(String.format("Yaw: %.2f degrees", yaw));
                    telemetry.addLine(String.format("Pitch: %.2f degrees", pitch));
                    telemetry.addLine(String.format("Roll: %.2f degrees", roll));

                    // Add range, bearing, and elevation for additional context
                    telemetry.addLine("\n==== Polar Coordinates ====");
                    telemetry.addLine(String.format("Range: %.2f inches", detection.ftcPose.range));
                    telemetry.addLine(String.format("Bearing: %.2f degrees", detection.ftcPose.bearing));
                    telemetry.addLine(String.format("Elevation: %.2f degrees", detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\nUnknown AprilTag (ID: %d)", detection.id));
                    telemetry.addLine(String.format("Center X: %.0f, Center Y: %.0f", detection.center.x, detection.center.y));
                }
            }

            // Allow toggling the camera stream
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            telemetry.addLine("\nCamera Controls:");
            telemetry.addLine("DPAD Up - Resume camera stream");
            telemetry.addLine("DPAD Down - Stop camera stream");

            telemetry.update();
            sleep(20); // Give other threads time to run
        }

        // Clean up
        visionPortal.close();
    }
}