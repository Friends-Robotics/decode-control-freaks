package org.firstinspires.ftc.teamcode.friends.helpers;

public class PIDController {
    private double kp;
    private double ki;
    private double kd;

    private double lastState = 0;
    private double integral = 0;
    private double lastTime = 0;

    private double integralLimit;
    private boolean firstRun = true;

    /**
     * Constructs a new PID controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    public PIDController(double kp, double ki, double kd) {
        this(kp, ki, kd, Double.MAX_VALUE);
    }

    /**
     * Constructs a new PID controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param integralLimit The maximum absolute value the integral term can reach
     */
    public PIDController(double kp, double ki, double kd, double integralLimit) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integralLimit = integralLimit;
    }

    /**
     * Calculates the control output based on the current system state.
     * * @param target The desired setpoint (where you want to be).
     * @param state  The current measured value (where you are now).
     * @return The calculated control output to be applied to the system.
     */
    public double calculate(double target, double state) {
        double now = System.nanoTime() / 1e9; // Divide to ensure the value is not massive

        // Initialization on the first loop to prevent 'dt' spikes
        if (firstRun) {
            lastTime = now;
            lastState = state;
            firstRun = false;
            return 0;
        }

        double dt = now - lastTime;

        if (dt <= 0) return 0;

        double error = target - state;
        double pTerm = kp * error;

        integral += error * dt;
        integral = Utils.clamp(integral, -integralLimit, integralLimit);
        double iTerm = ki * integral;

        double derivative = -(state - lastState) / dt;
        double dTerm = kd * derivative;

        lastTime = now;
        lastState = state;

        return pTerm + iTerm + dTerm;
    }

    /**
     * Resets the internal state (integral and timers).
     */
    public void reset() {
        integral = 0;
        firstRun = true;
    }
}
