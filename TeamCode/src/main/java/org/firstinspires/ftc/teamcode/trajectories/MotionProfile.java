package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.dashboard.config.Config;

/**
 * Handles motion profiling for smooth acceleration and deceleration
 * Based on trapezoidal motion profiles (accelerate, cruise, decelerate)
 */
@Config
public class MotionProfile {
    public static class Params {
        // Default constraints
        public static double maxVel = 1000.0;    // mm/s
        public static double maxAccel = 500.0;   // mm/s²
        public static double maxJerk = 250.0;    // mm/s³ (for smoothing)
    }

    private final double startPos;
    private final double endPos;
    private final double maxVel;
    private final double maxAccel;

    // Profile segments
    private double accelTime = 0;    // Time spent accelerating
    private double cruiseTime = 0;   // Time at max velocity
    private double decelTime = 0;    // Time spent decelerating
    private double totalTime = 0;    // Total profile duration

    public MotionProfile(double startPos, double endPos, double maxVel, double maxAccel) {
        this.startPos = startPos;
        this.endPos = endPos;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;

        calculateProfile();
    }

    private void calculateProfile() {
        double distance = Math.abs(endPos - startPos);

        // Time to reach max velocity
        double timeToMaxVel = maxVel / maxAccel;

        // Distance covered during acceleration/deceleration
        double accelDist = 0.5 * maxAccel * timeToMaxVel * timeToMaxVel;

        if (2 * accelDist <= distance) {
            // Normal case - we reach max velocity
            accelTime = timeToMaxVel;
            decelTime = timeToMaxVel;

            // Calculate cruise time
            double cruiseDist = distance - 2 * accelDist;
            cruiseTime = cruiseDist / maxVel;
        } else {
            // Short movement - we never reach max velocity
            // Solve for new acceleration time
            accelTime = Math.sqrt(distance / maxAccel);
            decelTime = accelTime;
            cruiseTime = 0;
        }

        totalTime = accelTime + cruiseTime + decelTime;
    }

    /**
     * Get target position at time t
     */
    public double getPosition(double t) {
        if (t <= 0) return startPos;
        if (t >= totalTime) return endPos;

        double pos;
        double direction = endPos >= startPos ? 1.0 : -1.0;

        if (t < accelTime) {
            // Acceleration phase
            pos = startPos + direction * (0.5 * maxAccel * t * t);
        } else if (t < accelTime + cruiseTime) {
            // Cruise phase
            double deltaT = t - accelTime;
            pos = startPos + direction * (
                    0.5 * maxAccel * accelTime * accelTime + // Distance from acceleration
                            maxVel * deltaT                          // Distance from cruise
            );
        } else {
            // Deceleration phase
            double deltaT = t - (accelTime + cruiseTime);
            double decelStart = startPos + direction * (
                    0.5 * maxAccel * accelTime * accelTime +
                            maxVel * cruiseTime
            );
            pos = decelStart + direction * (
                    maxVel * deltaT -
                            0.5 * maxAccel * deltaT * deltaT
            );
        }

        return pos;
    }

    /**
     * Get target velocity at time t
     */
    public double getVelocity(double t) {
        if (t <= 0 || t >= totalTime) return 0;

        double vel;
        double direction = endPos >= startPos ? 1.0 : -1.0;

        if (t < accelTime) {
            // Acceleration phase
            vel = direction * maxAccel * t;
        } else if (t < accelTime + cruiseTime) {
            // Cruise phase
            vel = direction * maxVel;
        } else {
            // Deceleration phase
            double deltaT = t - (accelTime + cruiseTime);
            vel = direction * (maxVel - maxAccel * deltaT);
        }

        return vel;
    }

    public double getTotalTime() {
        return totalTime;
    }
}