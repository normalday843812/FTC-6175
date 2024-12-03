package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import java.util.Locale;

@TeleOp(name="Blue Detection Tuning", group="Tuning")
public class BlueTuningOpmode extends LinearOpMode {
    private OpenCvCamera camera;

    static class BlueTuningPipeline extends OpenCvPipeline {
        private final Mat hsvMat = new Mat();
        private final Mat maskMat = new Mat();
        private final Mat hierarchyMat = new Mat();
        private final Mat outputMat = new Mat();

        private volatile double largestContourArea = 0;
        private final Point objectCenter = new Point();
        private volatile boolean blueObjectDetected = false;
        private double currentAngle = 0.0;

        // HSV Thresholds
        private int hMin = 165;
        private int hMax = 255;
        private int sMin = 150;
        private int sMax = 100;
        private int vMin = 50;
        private int vMax = 100;

        private boolean showMask = false;

        @Override
        public Mat processFrame(Mat input) {
            // Create copy for output
            input.copyTo(outputMat);

            // Convert to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Create mask with current thresholds
            Scalar lowerBlue = new Scalar(hMin, sMin, vMin);
            Scalar upperBlue = new Scalar(hMax, sMax, vMax);
            Core.inRange(hsvMat, lowerBlue, upperBlue, maskMat);

            // Find contours
            java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
            Imgproc.findContours(maskMat, contours, hierarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            largestContourArea = 0;
            blueObjectDetected = false;

            // Draw info on output
            if (showMask) {
                // If showing mask, convert to BGR so we can draw colored elements on it
                Imgproc.cvtColor(maskMat, outputMat, Imgproc.COLOR_GRAY2BGR);
            }

            // Process contours
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > largestContourArea && area > 500) {
                    largestContourArea = area;

                    MatOfPoint2f contour2f = new MatOfPoint2f();
                    contour.convertTo(contour2f, CvType.CV_32FC2);

                    RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
                    objectCenter.x = rotatedRect.center.x;
                    objectCenter.y = rotatedRect.center.y;

                    currentAngle = rotatedRect.angle;
                    if (rotatedRect.size.width < rotatedRect.size.height) {
                        currentAngle = -currentAngle;
                    } else {
                        currentAngle = 90 - currentAngle;
                    }

                    while (currentAngle > 90) currentAngle -= 180;
                    while (currentAngle < -90) currentAngle += 180;

                    // Draw rotated rectangle
                    Point[] vertices = new Point[4];
                    rotatedRect.points(vertices);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(outputMat, vertices[i], vertices[(i+1)%4],
                                showMask ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0), 2);
                    }

                    // Draw angle line
                    double radians = Math.toRadians(currentAngle);
                    Point endPoint = new Point(
                            objectCenter.x + Math.cos(radians) * 50,
                            objectCenter.y + Math.sin(radians) * 50
                    );
                    Imgproc.line(outputMat, objectCenter, endPoint,
                            showMask ? new Scalar(255, 0, 0) : new Scalar(0, 255, 0), 2);

                    blueObjectDetected = true;
                    contour2f.release();
                }
                contour.release();
            }

            // Draw threshold values and instructions
            String threshStr = String.format(Locale.US,
                    "H: %d-%d  S: %d-%d  V: %d-%d",
                    hMin, hMax, sMin, sMax, vMin, vMax);

            Imgproc.putText(outputMat, threshStr, new Point(10, 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);

            if (blueObjectDetected) {
                Imgproc.putText(outputMat,
                        String.format(Locale.US, "Angle: %.1f° Area: %.0f", currentAngle, largestContourArea),
                        new Point(10, 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
                        new Scalar(255, 255, 255), 1);
            }

            return outputMat;
        }

        public void incrementHMin() { hMin = Math.min(hMin + 5, hMax - 1); }
        public void decrementHMin() { hMin = Math.max(0, hMin - 5); }
        public void incrementHMax() { hMax = Math.min(180, hMax + 5); }
        public void decrementHMax() { hMax = Math.max(hMin + 1, hMax - 5); }

        public void incrementSMin() { sMin = Math.min(sMin + 5, sMax - 1); }
        public void decrementSMin() { sMin = Math.max(0, sMin - 5); }
        public void incrementSMax() { sMax = Math.min(255, sMax + 5); }
        public void decrementSMax() { sMax = Math.max(sMin + 1, sMax - 5); }

        public void incrementVMin() { vMin = Math.min(vMin + 5, vMax - 1); }
        public void decrementVMin() { vMin = Math.max(0, vMin - 5); }
        public void incrementVMax() { vMax = Math.min(255, vMax + 5); }
        public void decrementVMax() { vMax = Math.max(vMin + 1, vMax - 5); }

        public void toggleView() { showMask = !showMask; }

        public double getAngle() { return currentAngle; }
        public double getArea() { return largestContourArea; }
        public boolean isDetected() { return blueObjectDetected; }
        public String getHSVValues() {
            return String.format(Locale.US,
                    "H: [%d, %d]\nS: [%d, %d]\nV: [%d, %d]",
                    hMin, hMax, sMin, sMax, vMin, vMax);
        }
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        BlueTuningPipeline pipeline = new BlueTuningPipeline();
        camera.setPipeline(pipeline);

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.addLine("Controls:");
        telemetry.addLine("Dpad - Adjust Hue (left/right: min, up/down: max)");
        telemetry.addLine("Bumpers + Dpad - Adjust Saturation");
        telemetry.addLine("Triggers + Dpad - Adjust Value");
        telemetry.addLine("Y - Toggle mask view");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Hue adjustments
            if (gamepad1.dpad_left) pipeline.decrementHMin();
            if (gamepad1.dpad_right) pipeline.incrementHMin();
            if (gamepad1.dpad_down) pipeline.decrementHMax();
            if (gamepad1.dpad_up) pipeline.incrementHMax();

            // Saturation adjustments (with bumpers)
            if (gamepad1.left_bumper) {
                if (gamepad1.dpad_left) pipeline.decrementSMin();
                if (gamepad1.dpad_right) pipeline.incrementSMin();
                if (gamepad1.dpad_down) pipeline.decrementSMax();
                if (gamepad1.dpad_up) pipeline.incrementSMax();
            }

            // Value adjustments (with triggers)
            if (gamepad1.left_trigger > 0.5) {
                if (gamepad1.dpad_left) pipeline.decrementVMin();
                if (gamepad1.dpad_right) pipeline.incrementVMin();
                if (gamepad1.dpad_down) pipeline.decrementVMax();
                if (gamepad1.dpad_up) pipeline.incrementVMax();
            }

            // Toggle mask view
            if (gamepad1.y) pipeline.toggleView();

            telemetry.addLine("\nHSV Thresholds");
            telemetry.addData("Values", pipeline.getHSVValues());

            if (pipeline.isDetected()) {
                telemetry.addLine("\nDetection Info");
                telemetry.addData("Angle", String.format("%.1f°", pipeline.getAngle()));
                telemetry.addData("Area", String.format("%.0f pixels", pipeline.getArea()));
            } else {
                telemetry.addLine("\nNo object detected");
            }

            telemetry.update();

            sleep(100);  // Brief delay to prevent too rapid adjustments
        }

        if (camera != null) {
            camera.stopStreaming();
        }
    }
}