package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDController {

    private final double kP, kI, kD;
    private double errorSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    private double integralMin = Double.NEGATIVE_INFINITY;
    private double integralMax = Double.POSITIVE_INFINITY;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    // save last output for derivative smoothing
    private double lastOutput = 0;
    private static final double DERIVATIVE_SMOOTHING = 0.8;  // 0 = no smoothing, 1 = infinite smoothing

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
        lastOutput = 0;
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
     * Calculate PID output for given error
     * If dt is too large or zero, only P term is used
     */
    public double calculate(double error) {
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;

        // Prevent huge dt values or divide by zero
        if (dt <= 0 || dt > 0.5) {  // Skip I and D if dt > 0.5 sec (robot probably stuck/disabled)
            lastTime = currentTime;
            lastError = error;
            return kP * error;  // Just use P term
        }

        // Proportional term
        double P = kP * error;

        // Integral term with anti-windup
        // Only integrate if we're close to target (prevent windup from large errors)
        if (Math.abs(error) < 1.0) {  // Arbitrary threshold, adjust if needed
            errorSum += error * dt;
        }
        errorSum = Math.min(Math.max(errorSum, integralMin), integralMax);
        double I = kI * errorSum;

        // Derivative term on error (with smoothing)
        // Note: Some prefer derivative on measurement to avoid derivative kick
        double derivative = (error - lastError) / dt;
        double smoothedDerivative = DERIVATIVE_SMOOTHING * lastOutput +
                (1 - DERIVATIVE_SMOOTHING) * derivative;
        double D = kD * smoothedDerivative;

        // Update state
        lastError = error;
        lastTime = currentTime;

        // Calculate total output
        double output = P + I + D;
        lastOutput = output;

        // Limit output
        return Math.min(Math.max(output, outputMin), outputMax);
    }

}