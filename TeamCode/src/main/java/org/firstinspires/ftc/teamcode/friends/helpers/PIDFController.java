package org.firstinspires.ftc.teamcode.friends.helpers;

/**
 * PIDF Controller wrapper.
 * Adds a Feedforward component (kf) to the standard PID logic.
 */
public class PIDFController {
    private final PIDController pid;
    private double kf;

    /**
     * Constructs a new PIDF controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param kf Feedforward gain
     */
    public PIDFController(double kp, double ki, double kd, double kf) {
        this.pid = new PIDController(kp, ki, kd);
        this.kf = kf;
    }

    /**
     * Constructs a new PIDF controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param kf Feedforward gain
     * @param integralLimit The maximum absolute value the integral term can reach
     */
    public PIDFController(double kp, double ki, double kd, double kf, double integralLimit) {
        this.pid = new PIDController(kp, ki, kd, integralLimit);
        this.kf = kf;
    }

    /**
     * Calculates the control output based on the current system state
     * @param target The desired setpoint (where you want to be)
     * @param state  The current measured value (where you are now)
     * @return The calculated control output to be applied to the system
     */
    public double calculate(double target, double state) {
        double pidOutput = pid.calculate(target, state);

        double feedforward = target * kf;

        return pidOutput + feedforward;
    }

    /**
     * Resets the internal state (integral and timers).
     */
    public void reset() {
        pid.reset();
    }
}
