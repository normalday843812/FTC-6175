package org.firstinspires.ftc.teamcode.trajectories;

import org.firstinspires.ftc.teamcode.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;

import java.util.Arrays;

/**
 * SplinePath class for smooth trajectory generation.
 * Uses cubic splines for position interpolation.
 * Coordinate system:
 * - Positive X is forward
 * - Positive Y is right
 * - Positive heading is clockwise
 */
public class SplinePath {
    private final double[] xCoeffs;  // Coefficients for x(u) = a*u³ + b*u² + c*u + d
    private final double[] yCoeffs;  // Coefficients for y(u) = a*u³ + b*u² + c*u + d

    // TODO: might need to increase this if paths are too jerky
    private static final int NUM_SAMPLES = 1000; // Number of samples for lookup table
    private final double[] uSamples;  // Array of u values
    private final double[] sSamples;  // Corresponding s(u) values (cumulative arc length)
    private final double totalLength; // Total length of the spline

    public SplinePath(Pose2d startPose, Pose2d endPose) {
        // Calculate tangent vectors using pose headings
        // Note: For our coordinate system, positive heading is clockwise
        Vector2d startTangent = new Vector2d(
                Math.cos(startPose.heading),
                Math.sin(startPose.heading)
        );
        Vector2d endTangent = new Vector2d(
                Math.cos(endPose.heading),
                Math.sin(endPose.heading)
        );

        // Scale tangents by path length for better shape control
        double distance = endPose.position.minus(startPose.position).norm();
        startTangent = startTangent.times(distance);
        endTangent = endTangent.times(distance);

        // Fit x(u) spline
        xCoeffs = fitCubic(
                startPose.position.x,    // start x
                startTangent.x,          // start dx/du
                endPose.position.x,      // end x
                endTangent.x             // end dx/du
        );

        // Fit y(u) spline
        yCoeffs = fitCubic(
                startPose.position.y,    // start y
                startTangent.y,          // start dy/du
                endPose.position.y,      // end y
                endTangent.y             // end dy/du
        );

        // Initialize arrays for lookup table
        uSamples = new double[NUM_SAMPLES + 1];
        sSamples = new double[NUM_SAMPLES + 1];

        // TODO: this lookup table stuff is sketchy but it works
        // if the robot does weird speed stuff on curves check here first
        double du = 1.0 / NUM_SAMPLES;
        double s = 0.0;
        sSamples[0] = 0.0;
        uSamples[0] = 0.0;

        // Previous point (x, y)
        double prevX = getX(0.0);
        double prevY = getY(0.0);

        // Build lookup table for arc length parameterization
        // i spent way too long on this math i stg if it breaks
        for (int i = 1; i <= NUM_SAMPLES; i++) {
            double u = i * du;
            uSamples[i] = u;

            // Current point (x, y)
            double currX = getX(u);
            double currY = getY(u);

            // Compute distance between previous and current points
            double dx = currX - prevX;
            double dy = currY - prevY;
            double ds = Math.hypot(dx, dy);

            // Cumulative arc length
            s += ds;
            sSamples[i] = s;

            // Update previous point
            prevX = currX;
            prevY = currY;
        }

        totalLength = s;
    }

    /**
     * Fit cubic coefficients given endpoints and derivatives.
     * Returns coefficients [a, b, c, d] for a*u³ + b*u² + c*u + d
     *
     * TODO: don't touch this unless you want to solve a 4x4 matrix by hand
     */
    private double[] fitCubic(double start, double startDeriv, double end, double endDeriv) {
        return new double[]{
                2 * start + startDeriv - 2 * end + endDeriv,    // a
                -3 * start - 2 * startDeriv + 3 * end - endDeriv, // b
                startDeriv,                                     // c
                start                                           // d
        };
    }

    /**
     * Get x-coordinate at parameter u (0 ≤ u ≤ 1)
     */
    private double getX(double u) {
        return xCoeffs[0] * u * u * u + xCoeffs[1] * u * u + xCoeffs[2] * u + xCoeffs[3];
    }

    /**
     * Get y-coordinate at parameter u (0 ≤ u ≤ 1)
     */
    private double getY(double u) {
        return yCoeffs[0] * u * u * u + yCoeffs[1] * u * u + yCoeffs[2] * u + yCoeffs[3];
    }

    /**
     * Get derivative dx/du at parameter u
     */
    private double getdX(double u) {
        return 3 * xCoeffs[0] * u * u + 2 * xCoeffs[1] * u + xCoeffs[2];
    }

    /**
     * Get derivative dy/du at parameter u
     */
    private double getdY(double u) {
        return 3 * yCoeffs[0] * u * u + 2 * yCoeffs[1] * u + yCoeffs[2];
    }

    /**
     * Get cumulative displacement s(u) for a given u
     */
    // TODO: binary search might be overkill but whatever
    public double getDisplacement(double u) {
        if (u <= 0.0) return 0.0;
        if (u >= 1.0) return totalLength;

        int index = Arrays.binarySearch(uSamples, u);
        if (index >= 0) {
            return sSamples[index];
        }

        // Linear interpolation if exact value not found
        index = -index - 1;
        if (index == 0) return sSamples[0];
        if (index >= uSamples.length) return sSamples[sSamples.length - 1];

        double u0 = uSamples[index - 1];
        double u1 = uSamples[index];
        double s0 = sSamples[index - 1];
        double s1 = sSamples[index];

        double ratio = (u - u0) / (u1 - u0);
        return s0 + ratio * (s1 - s0);
    }

    /**
     * Get parameter u(s) for a given displacement s
     */
    public double getParameter(double s) {
        if (s <= 0.0) return 0.0;
        if (s >= totalLength) return 1.0;

        // Binary search for s
        int index = Arrays.binarySearch(sSamples, s);
        if (index >= 0) {
            return uSamples[index];
        }

        // Linear interpolation
        index = -index - 1;
        if (index == 0) return uSamples[0];
        if (index >= sSamples.length) return uSamples[uSamples.length - 1];

        double s0 = sSamples[index - 1];
        double s1 = sSamples[index];
        double u0 = uSamples[index - 1];
        double u1 = uSamples[index];

        // TODO: this ratio stuff is everywhere, might want to check the math
        double ratio = (s - s0) / (s1 - s0);
        return u0 + ratio * (u1 - u0);
    }

    /**
     * Get position along the spline at parameter u (0 ≤ u ≤ 1)
     */
    public Vector2d getPosition(double u) {
        if (u < 0) u = 0;
        if (u > 1) u = 1;

        double x = getX(u);
        double y = getY(u);

        return new Vector2d(x, y);
    }

    /**
     * Get velocity (dx/du, dy/du) at parameter u
     */
    public Vector2d getVelocity(double u) {
        double dx = getdX(u);
        double dy = getdY(u);
        return new Vector2d(dx, dy);
    }

    /**
     * Get heading (angle of velocity vector) at parameter u
     */
    public double getHeading(double u) {
        return Math.atan2(getdY(u), getdX(u));
    }

    /**
     * Get the pose (position and heading) at a given displacement
     */
    public Pose2d getPose(double displacement) {
        double u = getParameter(displacement);
        Vector2d position = getPosition(u);
        double heading = getHeading(u);
        return new Pose2d(position, heading);
    }

    /**
     * Get the total path length (for motion profiling)
     */
    public double getLength() {
        return totalLength;
    }
}