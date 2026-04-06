package org.firstinspires.ftc.teamcode.friends.helpers;

/**
 * PIDF Controller wrapper.
 * Adds a Feedforward component (kf) to the standard PID logic.
 */
public class PIDFController extends PIDController {
    private double ks; // Static friction
    private double kv; // Velocity

    /**
     * Constructs a new PIDF controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ks Feedforward gain (static, e.g. a constant static feedforward resistance such as friction)
     * @param kv Feedforward gain (proportional, e.g. a resistance proportional to the target)
     * @param integralLimit The maximum absolute value the integral term can reach
     */
    public PIDFController(double kp, double ki, double kd, double ks, double kv, double integralLimit) {
        super(kp, ki, kd, integralLimit);
        this.ks = ks;
        this.kv = kv;
    }

    /**
     * Calculates the control output based on the current system state
     * @param target The desired setpoint (where you want to be)
     * @param state  The current measured value (where you are now)
     * @return The calculated control output to be applied to the system
     */
    @Override
    public double calculate(double target, double state) {
        // Use the parent's calculate method to get the base PID output
        double pidOutput = super.calculate(target, state);

        // Calculate Feedforward terms
        double error = target - state;
        double staticFF = Math.signum(error) * ks;
        double velocityFF = target * kv;

        return pidOutput + staticFF + velocityFF;
    }

    public void setPIDF(double kp, double ki, double kd, double ks, double kv) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kv = kv;
    }

    public void setKs(double ks) { this.ks = ks; }
    public void setKv(double kv) { this.kv = kv; }
}