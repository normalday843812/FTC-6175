package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class BlueDetectionPipeline extends OpenCvPipeline {
    private final Mat hsvMat = new Mat();
    private final Mat maskMat = new Mat();
    private final Mat hierarchyMat = new Mat();
    private final Mat drawingMat = new Mat();
    private volatile double largestContourArea = 0;
    private final Point objectCenter = new Point();
    private volatile boolean blueObjectDetected = false;
    private double currentAngle = 0.0;
    private double rectangleWidth = 0.0;
    private double rectangleHeight = 0.0;

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(drawingMat);
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Define blue color range in HSV
        Scalar lowerBlue = new Scalar(100, 150, 50);
        Scalar upperBlue = new Scalar(140, 255, 255);
        Core.inRange(hsvMat, lowerBlue, upperBlue, maskMat);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(maskMat, contours, hierarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        largestContourArea = 0;
        blueObjectDetected = false;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea && area > 500) {  // Minimum area threshold
                largestContourArea = area;

                // Convert contour to MatOfPoint2f for rotated rectangle calculation
                MatOfPoint2f contour2f = new MatOfPoint2f();
                contour.convertTo(contour2f, CvType.CV_32FC2);

                // Calculate rotated rectangle
                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
                objectCenter.x = rotatedRect.center.x;
                objectCenter.y = rotatedRect.center.y;
                rectangleWidth = rotatedRect.size.width;
                rectangleHeight = rotatedRect.size.height;

                // Calculate angle
                currentAngle = rotatedRect.angle;
                if (rotatedRect.size.width < rotatedRect.size.height) {
                    currentAngle = -currentAngle;
                } else {
                    currentAngle = 90 - currentAngle;
                }

                // Normalize angle to -90 to 90 range
                while (currentAngle > 90) currentAngle -= 180;
                while (currentAngle < -90) currentAngle += 180;

                // Draw rotated rectangle
                Point[] vertices = new Point[4];
                rotatedRect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(drawingMat, vertices[i], vertices[(i+1)%4], new Scalar(0, 255, 0), 2);
                }

                // Draw angle line
                double radians = Math.toRadians(currentAngle);
                Point endPoint = new Point(
                        objectCenter.x + Math.cos(radians) * 50,
                        objectCenter.y + Math.sin(radians) * 50
                );
                Imgproc.line(drawingMat, objectCenter, endPoint, new Scalar(255, 0, 0), 2);

                blueObjectDetected = true;
                contour2f.release();
            }
            contour.release();
        }

        // Draw angle text if object is detected
        if (blueObjectDetected) {
            String angleText = String.format(Locale.US, "Angle: %.1f°", currentAngle);
            Imgproc.putText(
                    drawingMat,
                    angleText,
                    new Point(10, 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    new Scalar(255, 0, 0),
                    2
            );
        }

        return drawingMat;
    }

    // Getter methods
    public boolean isBlueObjectDetected() {
        return blueObjectDetected;
    }

    public double getLargestContourArea() {
        return largestContourArea;
    }

    public Point getObjectCenter() {
        return objectCenter;
    }

    public double getAngle() {
        return currentAngle;
    }

    public double getRectangleWidth() {
        return rectangleWidth;
    }

    public double getRectangleHeight() {
        return rectangleHeight;
    }

    // Release resources
    public void release() {
        hsvMat.release();
        maskMat.release();
        hierarchyMat.release();
        drawingMat.release();
    }
}