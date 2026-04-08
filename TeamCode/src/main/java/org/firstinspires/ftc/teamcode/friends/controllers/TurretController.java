package org.firstinspires.ftc.teamcode.friends.controllers;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;

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
     * @param error Degrees off target
     * @param distance Current distance to target
     * @return Motor power clipped to safe limits.
     */
    public double update(double error, double distance) {
        double scheduledKP = interpolateGain(distance);
        pidf.setkP(scheduledKP);

        return pidf.calculate(0, error);
    }

    /**
     * Logic to decide the PID strength based on distance.
     */
    private double interpolateGain(double distance) {
        double t = Range.clip((distance - 20) / (80 - 20), 0, 1);

        return RobotConstants.Turret.kP_CLOSE + t * (RobotConstants.Turret.kP_FAR - RobotConstants.Turret.kP_CLOSE);
    }

    /**
     * Checks if the turret is aligned
     * @param error
     * @return
     */
    public boolean isAligned(double error) {
        return Math.abs(error) < RobotConstants.Turret.ALIGN_TOLERANCE;
    }
}