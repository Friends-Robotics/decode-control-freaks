package org.firstinspires.ftc.teamcode.friends.controllers;

import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.friends.helpers.Utils;

public class TurretController {
    private final PIDFController pidf;

    private double currentPower = 0;
    private double lastError = 0;

    public TurretController() {
        this.pidf = new PIDFController(
                RobotConstants.Turret.kP,
                RobotConstants.Turret.kI,
                RobotConstants.Turret.kD,
                RobotConstants.Turret.kS,
                RobotConstants.Turret.kV,
                0.0,
                0.0,
                RobotConstants.Turret.iLimit
        );
        pidf.setTolerance(RobotConstants.Turret.ALIGN_TOLERANCE);
        pidf.setOutputBounds(-RobotConstants.Turret.MAX_POWER, RobotConstants.Turret.MAX_POWER);
    }

    /**
     * The core execution logic.
     * @param degreesFromTarget Current error in degrees
     * @return Motor power clipped to safe limits.
     */
    public double update(double degreesFromTarget) {
        this.lastError = degreesFromTarget;

        this.currentPower = pidf.calculate(0, degreesFromTarget);

        return currentPower;
    }

    /**
     * Instantaneous check if the turret is within the allowed tolerance.
     */
    public boolean isAligned() {
        return Utils.withinTolerance(lastError, 0, RobotConstants.Turret.ALIGN_TOLERANCE);
    }

    public double getCurrentPower() { return currentPower; }
    public double getLastError() { return lastError; }

    public void reset() {
        pidf.reset();
        currentPower = 0;
        lastError = 0;
    }

    public void updateConstants() {
        pidf.setPIDF(
                RobotConstants.Turret.kP,
                RobotConstants.Turret.kI,
                RobotConstants.Turret.kD,
                RobotConstants.Turret.kS,
                RobotConstants.Turret.kV,
                0.0,
                0.0
        );
        pidf.setTolerance(RobotConstants.Turret.ALIGN_TOLERANCE);
        pidf.setOutputBounds(-RobotConstants.Turret.MAX_POWER, RobotConstants.Turret.MAX_POWER);
    }
}