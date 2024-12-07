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
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class SampleDetectionPipeline extends OpenCvPipeline {
    // Color ranges in HSV
    private static final Scalar LOWER_BLUE = new Scalar(100, 100, 100);
    private static final Scalar UPPER_BLUE = new Scalar(130, 255, 255);
    private static final Scalar LOWER_YELLOW = new Scalar(20, 100, 100);
    private static final Scalar UPPER_YELLOW = new Scalar(30, 255, 255);
    private static final Scalar LOWER_RED1 = new Scalar(0, 100, 100);
    private static final Scalar UPPER_RED1 = new Scalar(10, 255, 255);
    private static final Scalar LOWER_RED2 = new Scalar(170, 100, 100);
    private static final Scalar UPPER_RED2 = new Scalar(180, 255, 255);

    // Detection parameters
    private static final double MIN_AREA = 500;
    private static final double MIN_ASPECT_RATIO = 1.8;
    private static final double MAX_ASPECT_RATIO = 2.2;
    private static final int SMOOTH_WINDOW_SIZE = 5;

    // Confidence parameters
    private static final double CONFIDENCE_LOCK_THRESHOLD = 0.8;
    private static final double CONFIDENCE_UNLOCK_THRESHOLD = 0.3;
    private static final double ANGLE_STABILITY_THRESHOLD = 2.0;
    private static final int STABLE_FRAMES_FOR_LOCK = 10;

    public enum ColorPair {
        YELLOW_BLUE,
        RED_YELLOW
    }

    // Processing objects
    private final ColorPair colorPair;
    private final Mat hsvMat = new Mat();
    private final Mat maskMat1 = new Mat();
    private final Mat maskMat2 = new Mat();

    // State tracking
    private Point objectCenter = new Point();
    private double largestContourArea = 0;
    private boolean sampleDetected = false;
    private String detectedColor = "None";

    // Angle tracking
    private final Queue<Double> recentAngles = new LinkedList<>();
    private double currentAngle = 0;
    private double smoothedAngle = 0;
    private boolean isLocked = false;
    private double lockedAngle = 0;
    private double currentConfidence = 0;
    private int stableFrameCount = 0;

    public SampleDetectionPipeline(ColorPair colorPair) {
        this.colorPair = colorPair;
    }

    /** @noinspection unused*/
    public SampleDetectionPipeline() {
        this(ColorPair.YELLOW_BLUE);
    }

    @Override
    public Mat processFrame(Mat input) {
        // Reset state
        sampleDetected = false;
        largestContourArea = 0;

        // Convert to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Find contours for both colors
        List<MatOfPoint> contours = new ArrayList<>();
        findContours(contours);

        // Process contours
        processContours(contours);

        // Clean up
        for (MatOfPoint contour : contours) {
            contour.release();
        }

        return input;
    }

    private void findContours(List<MatOfPoint> contours) {
        switch (colorPair) {
            case YELLOW_BLUE:
                Core.inRange(hsvMat, LOWER_YELLOW, UPPER_YELLOW, maskMat1);
                Imgproc.findContours(maskMat1, contours, new Mat(),
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                Core.inRange(hsvMat, LOWER_BLUE, UPPER_BLUE, maskMat2);
                Imgproc.findContours(maskMat2, contours, new Mat(),
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                break;

            case RED_YELLOW:
                Core.inRange(hsvMat, LOWER_YELLOW, UPPER_YELLOW, maskMat1);
                Imgproc.findContours(maskMat1, contours, new Mat(),
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                Mat redMask1 = new Mat();
                Mat redMask2 = new Mat();
                Core.inRange(hsvMat, LOWER_RED1, UPPER_RED1, redMask1);
                Core.inRange(hsvMat, LOWER_RED2, UPPER_RED2, redMask2);
                Core.bitwise_or(redMask1, redMask2, maskMat2);
                Imgproc.findContours(maskMat2, contours, new Mat(),
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                redMask1.release();
                redMask2.release();
                break;
        }
    }

    private void processContours(List<MatOfPoint> contours) {
        MatOfPoint bestContour = null;
        double bestScore = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_AREA && isValidShape(contour)) {
                double score = calculateScore(contour);
                if (score > bestScore) {
                    bestScore = score;
                    bestContour = contour;
                    largestContourArea = area;
                }
            }
        }

        if (bestContour != null) {
            processSelectedContour(bestContour);
        } else {
            handleNoDetection();
        }
    }

    private boolean isValidShape(MatOfPoint contour) {
        MatOfPoint2f contour2f = new MatOfPoint2f();
        contour.convertTo(contour2f, CvType.CV_32FC2);
        RotatedRect rect = Imgproc.minAreaRect(contour2f);
        double ratio = Math.max(rect.size.width, rect.size.height) /
                Math.min(rect.size.width, rect.size.height);
        contour2f.release();
        return ratio >= MIN_ASPECT_RATIO && ratio <= MAX_ASPECT_RATIO;
    }

    private double calculateScore(MatOfPoint contour) {
        double area = Imgproc.contourArea(contour);
        double shapeConfidence = getShapeConfidence(contour);
        return area * shapeConfidence;
    }

    private void processSelectedContour(MatOfPoint contour) {
        sampleDetected = true;

        // Get rectangle and center
        MatOfPoint2f contour2f = new MatOfPoint2f();
        contour.convertTo(contour2f, CvType.CV_32FC2);
        RotatedRect rect = Imgproc.minAreaRect(contour2f);
        objectCenter = rect.center;

        // Calculate angle
        double rawAngle = rect.angle;
        if (rect.size.width < rect.size.height) {
            rawAngle += 90;
        }
        // Normalize to -90 to 90
        while (rawAngle > 90) rawAngle -= 180;
        while (rawAngle < -90) rawAngle += 180;

        // Update angle tracking
        currentAngle = rawAngle;
        updateAngleTracking();

        // Update confidence
        currentConfidence = calculateConfidence(contour);
        updateLockState();

        // Identify color
        detectedColor = determineColor(contour);

        contour2f.release();
    }

    private void updateAngleTracking() {
        recentAngles.add(currentAngle);
        while (recentAngles.size() > SMOOTH_WINDOW_SIZE) {
            recentAngles.remove();
        }

        if (!isLocked) {
            // Calculate smoothed angle using circular mean
            double sumSin = 0, sumCos = 0;
            for (double angle : recentAngles) {
                double rad = Math.toRadians(angle);
                sumSin += Math.sin(rad);
                sumCos += Math.cos(rad);
            }
            smoothedAngle = Math.toDegrees(Math.atan2(sumSin, sumCos));
        }
    }

    private double calculateConfidence(MatOfPoint contour) {
        double area = Imgproc.contourArea(contour);
        double shapeConfidence = getShapeConfidence(contour);
        double angleStability = getAngleStability();

        return (shapeConfidence * 0.4 +
                Math.min(1.0, area / 5000.0) * 0.3 +
                angleStability * 0.3);
    }

    private double getShapeConfidence(MatOfPoint contour) {
        MatOfPoint2f contour2f = new MatOfPoint2f();
        contour.convertTo(contour2f, CvType.CV_32FC2);
        RotatedRect rect = Imgproc.minAreaRect(contour2f);
        double aspectRatio = Math.max(rect.size.width, rect.size.height) /
                Math.min(rect.size.width, rect.size.height);
        contour2f.release();
        return Math.max(0, 1 - Math.abs(aspectRatio - 2.0));
    }

    private double getAngleStability() {
        if (recentAngles.size() < 2) return 0;

        Double firstAngle = recentAngles.peek();
        if (firstAngle == null) return 0;

        double maxDiff = 0;
        double lastAngle = firstAngle;

        for (double angle : recentAngles) {
            maxDiff = Math.max(maxDiff, Math.abs(angle - lastAngle));
            lastAngle = angle;
        }
        return Math.max(0, 1 - maxDiff / ANGLE_STABILITY_THRESHOLD);
    }

    private void updateLockState() {
        if (!isLocked) {
            if (currentConfidence > CONFIDENCE_LOCK_THRESHOLD) {
                if (Math.abs(currentAngle - smoothedAngle) < ANGLE_STABILITY_THRESHOLD) {
                    stableFrameCount++;
                    if (stableFrameCount >= STABLE_FRAMES_FOR_LOCK) {
                        isLocked = true;
                        lockedAngle = smoothedAngle;
                        stableFrameCount = 0;
                    }
                } else {
                    stableFrameCount = 0;
                }
            }
        } else if (currentConfidence < CONFIDENCE_UNLOCK_THRESHOLD) {
            isLocked = false;
            stableFrameCount = 0;
        }
    }

    private String determineColor(MatOfPoint contour) {
        // Create a mask from the contour
        Mat mask = Mat.zeros(hsvMat.size(), CvType.CV_8UC1);
        List<MatOfPoint> contours = new ArrayList<>();
        contours.add(contour);
        Imgproc.drawContours(mask, contours, 0, new Scalar(255), -1);

        // Calculate mean color in the contour area
        Scalar meanHsv = Core.mean(hsvMat, mask);
        mask.release();

        double hue = meanHsv.val[0];
        if (colorPair == ColorPair.YELLOW_BLUE) {
            return (hue >= 100 && hue <= 130) ? "Blue" : "Yellow";
        } else {
            return (hue <= 10 || hue >= 170) ? "Red" : "Yellow";
        }
    }

    private void handleNoDetection() {
        sampleDetected = false;
        currentConfidence = 0;
        isLocked = false;
        stableFrameCount = 0;
    }

    // Getters
    public boolean isSampleDetected() { return sampleDetected; }
    public double getConfidence() { return currentConfidence; }
    public boolean isLocked() { return isLocked; }
    public double getAngle() { return isLocked ? lockedAngle : smoothedAngle; }
    public String getColor() { return detectedColor; }
    /** @noinspection unused*/
    public Point getCenter() { return objectCenter; }
    public double getArea() { return largestContourArea; }
}