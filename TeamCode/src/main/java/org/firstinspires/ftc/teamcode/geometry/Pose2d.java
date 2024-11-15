package org.firstinspires.ftc.teamcode.geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * Represents a 2D pose (position and heading).
 * Used for representing robot state and trajectory targets.
 */
public class Pose2d {
    public final Vector2d position;
    public final double heading;

    public Pose2d() {
        this(0, 0, 0);
    }

    public Pose2d(double x, double y, double heading) {
        this.position = new Vector2d(x, y);
        this.heading = heading;
    }

    public Pose2d(Vector2d position, double heading) {
        this.position = position;
        this.heading = heading;
    }

    /**
     * Transforms this pose by another pose
     * Applies rotation and translation
     */
    public Pose2d plus(Pose2d other) {
        return new Pose2d(
                position.plus(other.position.rotated(heading)),
                heading + other.heading
        );
    }

    /**
     * Returns the difference between this pose and another pose
     */
    public Pose2d minus(Pose2d other) {
        Vector2d deltaPosition = position.minus(other.position);
        deltaPosition = deltaPosition.rotated(-other.heading);
        return new Pose2d(
                deltaPosition,
                heading - other.heading
        );
    }

    /**
     * Returns the error between this pose and a target pose
     * Used for trajectory following
     */
    public Pose2d getError(Pose2d targetPose) {
        Pose2d error = targetPose.minus(this);
        return new Pose2d(
                error.position,
                Math.atan2(Math.sin(error.heading), Math.cos(error.heading)) // Normalize angle
        );
    }

    /**
     * Returns heading error normalized to [-π, π]
     */
    public double getHeadingError(Pose2d targetPose) {
        double error = targetPose.heading - heading;
        return Math.atan2(Math.sin(error), Math.cos(error));
    }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        return String.format("<%.3f, %.3f, %.3f°>",
                position.x, position.y, Math.toDegrees(heading));
    }
}