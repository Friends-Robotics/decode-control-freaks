package org.firstinspires.ftc.teamcode.friends.helpers;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
    // PID Constants
    protected double kP;
    protected double kI;
    protected double kD;

    // State Variables
    protected double lastState = 0;
    protected double integral = 0;
    protected double lastTime = 0;
    protected boolean firstRun = true;

    // Constraints
    protected double integralLimit;
    protected double minOutput = -1.0;
    protected double maxOutput = 1.0;
    protected double targetTolerance = 0.0;

    public PIDController(double kp, double ki, double kd) {
        this(kp, ki, kd, Double.MAX_VALUE);
    }

    public PIDController(double kp, double ki, double kd, double integralLimit) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.integralLimit = integralLimit;
    }

    /**
     * Calculates output of the PID controller
     */
    public double calculate(double target, double state) {
        double now = System.nanoTime() / 1e9;

        if (firstRun) {
            lastTime = now;
            lastState = state;
            firstRun = false;
            return 0.0;
        }

        double dt = now - lastTime;
        if (dt <= 0) return 0;

        double error = target - state;

        // Deadband check
        if (Math.abs(error) <= targetTolerance) {
            return 0.0;
        }

        double pTerm = kP * error;

        integral += error * dt;
        integral = Range.clip(integral, -integralLimit, integralLimit);
        double iTerm = kI * integral;

        double derivative = -(state - lastState) / dt;
        double dTerm = kD * derivative;

        lastTime = now;
        lastState = state;

        double output = pTerm + iTerm + dTerm;
        return Range.clip(output, minOutput, maxOutput);
    }

    public void reset() {
        integral = 0;
        firstRun = true;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setkP(double kP) { this.kP = kP; }
    public void setkI(double kI) { this.kI = kI; }
    public void setkD(double kD) { this.kD = kD; }
    public void setIntegralLimit(double limit) { this.integralLimit = limit; }
    public void setTolerance(double tolerance) { this.targetTolerance = tolerance; }
    public void setOutputBounds(double min, double max) { this.minOutput = min; this.maxOutput = max; }
}