package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDController {
    public static class Params {
        // Default gains - can be tuned via Dashboard
        public static double DEFAULT_kP = 0.1;
        public static double DEFAULT_kI = 0.0;
        public static double DEFAULT_kD = 0.0;
    }

    private final double kP, kI, kD;
    private double errorSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    private double integralMin = Double.NEGATIVE_INFINITY;
    private double integralMax = Double.POSITIVE_INFINITY;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    public PIDController() {
        this(Params.DEFAULT_kP, Params.DEFAULT_kI, Params.DEFAULT_kD);
    }

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Reset the controller's integral and derivative terms
     */
    public void reset() {
        errorSum = 0;
        lastError = 0;
        lastTime = System.nanoTime() / 1e9;
    }

    /**
     * Set limits on the integral term to prevent windup
     */
    public void setIntegralLimits(double min, double max) {
        this.integralMin = min;
        this.integralMax = max;
    }

    /**
     * Set limits on the controller output
     */
    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    /**
     * Calculate control output based on error and elapsed time
     */
    public double calculate(double error) {
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;

        if (dt <= 0) {
            lastTime = currentTime;
            return 0;
        }

        // Proportional term
        double P = kP * error;

        // Integral term with anti-windup
        errorSum += error * dt;
        errorSum = Math.min(Math.max(errorSum, integralMin), integralMax);
        double I = kI * errorSum;

        // Derivative term (on measurement to avoid derivative kick)
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        double D = kD * derivative;

        // Update state
        lastError = error;
        lastTime = currentTime;

        // Calculate and limit output
        double output = P + I + D;
        return Math.min(Math.max(output, outputMin), outputMax);
    }

    /**
     * Calculate control output for tracking a target value
     */
    public double calculate(double current, double target) {
        return calculate(target - current);
    }
}