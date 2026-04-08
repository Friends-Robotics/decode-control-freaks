package org.firstinspires.ftc.teamcode.friends.helpers;

import com.qualcomm.robotcore.util.Range;

/**
 * PIDFController inherits from PIDController and adds Feedforward components.
 * Best used for mechanisms requiring predictive power like Flywheels, Lifts, and Arms.
 */
public class PIDFController extends PIDController {
    // Feedforward Constants
    protected double kS; // Static friction (minimum power to break stiction)
    protected double kV; // Velocity (power to maintain constant speed)
    protected double kA; // Acceleration (power to change speed)
    protected double kG; // Gravity (power to hold position against a constant force)

    /**
     * Constructs a new PIDF controller with full constants.
     */
    public PIDFController(double kP, double kI, double kD, double kS, double kV, double kA, double kG, double integralLimit) {
        super(kP, kI, kD, integralLimit);
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    @Override
    public double calculate(double target, double state) {
        return calculate(target, state, 0, 0);
    }

    /**
     * Full PIDF calculation.
     * @param target Desired setpoint (Position/RPM)
     * @param state Current measured value
     * @param targetVel The target velocity (from a motion profile)
     * @param targetAccel The target acceleration (from a motion profile)
     */
    public double calculate(double target, double state, double targetVel, double targetAccel) {
        double pidOutput = super.calculate(target, state);

        if (pidOutput == 0 && Math.abs(target - state) <= targetTolerance) {
            return 0.0;
        }

        double staticFF = Math.signum(target - state) * kS;

        double velocityFF = targetVel * kV;
        double accelerationFF = targetAccel * kA;

        double gravityFF = kG;

        double totalOutput = pidOutput + staticFF + velocityFF + accelerationFF + gravityFF;

        return Range.clip(totalOutput, minOutput, maxOutput);
    }

    public void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        setPID(kP, kI, kD);
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    public void setkS(double kS) { this.kS = kS; }
    public void setkV(double kV) { this.kV = kV; }
    public void setkA(double kA) { this.kA = kA; }
    public void setkG(double kG) { this.kG = kG; }
}