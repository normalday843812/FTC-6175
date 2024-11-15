package org.firstinspires.ftc.teamcode.geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * Represents a vector in 2D space with x and y components.
 * Used for representing positions and velocities.
 */
public class Vector2d {
    public final double x;
    public final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Adds this vector with another vector
     */
    public Vector2d plus(Vector2d other) {
        return new Vector2d(x + other.x, y + other.y);
    }

    /**
     * Subtracts another vector from this vector
     */
    public Vector2d minus(Vector2d other) {
        return new Vector2d(x - other.x, y - other.y);
    }

    /**
     * Multiplies this vector by a scalar
     */
    public Vector2d times(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }

    /**
     * Returns the magnitude (length) of this vector
     */
    public double norm() {
        return Math.hypot(x, y);
    }

    /**
     * Returns the angle between this vector and the positive x-axis
     */
    public double angle() {
        return Math.atan2(y, x);
    }

    /**
     * Returns a vector rotated by the specified angle in radians
     */
    public Vector2d rotated(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2d(
                x * cos - y * sin,
                x * sin + y * cos
        );
    }

    /**
     * Returns a normalized version of this vector (magnitude = 1)
     */
    public Vector2d normalized() {
        double norm = norm();
        if (norm == 0) return new Vector2d(0, 0);
        return this.times(1.0 / norm);
    }

    /**
     * Returns the dot product of this vector with another vector
     */
    public double dot(Vector2d other) {
        return x * other.x + y * other.y;
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("<%.3f, %.3f>", x, y);
    }
}