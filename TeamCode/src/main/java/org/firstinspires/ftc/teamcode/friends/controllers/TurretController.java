package org.firstinspires.ftc.teamcode.friends.controllers;

import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.friends.helpers.Utils;

public class TurretController {
    private final PIDFController pidf = new PIDFController(
            RobotConstants.Turret.kP,
            RobotConstants.Turret.kI,
            RobotConstants.Turret.kD,
            RobotConstants.Turret.kS,
            RobotConstants.Turret.kV,
            0.0,
            0.0,
            RobotConstants.Turret.iLimit
    );

    public TurretController() {
        pidf.setTolerance(RobotConstants.Turret.ALIGN_TOLERANCE);
        pidf.setOutputBounds(-RobotConstants.Turret.MAX_POWER, RobotConstants.Turret.MAX_POWER);
    }

    public void reset() {
        pidf.reset();
    }

    /**
     * The core execution logic.
     * @param degreesFromTarget Degrees off target
     * @param distance Current distance to target
     * @return Motor power clipped to safe limits.
     */
    public double update(double degreesFromTarget, double distance) {
        double scheduledKP = interpolateGain(distance);
        pidf.setkP(scheduledKP);

        return pidf.calculate(0, degreesFromTarget);
    }

    /**
     * Logic to decide the PID strength based on distance.
     */
    private double interpolateGain(double distance) {
        double t = Utils.getT(distance, RobotConstants.Turret.MIN_TRACKING_DISTANCE, RobotConstants.Turret.MAX_TRACKING_DISTANCE);

        return Utils.lerp(RobotConstants.Turret.kP_CLOSE, RobotConstants.Turret.kP_FAR, t);
    }

    /**
     * Checks if the turret is aligned
     * @param degreesFromTarget
     * @return
     */
    public boolean isAligned(double degreesFromTarget) {
        return Math.abs(degreesFromTarget) < RobotConstants.Turret.ALIGN_TOLERANCE;
    }
}