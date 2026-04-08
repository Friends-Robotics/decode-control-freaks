package org.firstinspires.ftc.teamcode.friends.controllers;

import org.firstinspires.ftc.teamcode.friends.helpers.LowPassFilter;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.friends.helpers.Utils;

public class ShooterController {
    private final LowPassFilter rpmFilter;
    private final PIDFController pidf = new PIDFController(
            RobotConstants.Shooter.kP,
            RobotConstants.Shooter.kI,
            RobotConstants.Shooter.kD,
            RobotConstants.Shooter.kS,
            RobotConstants.Shooter.kV,
            0.0,
            0.0,
            RobotConstants.Shooter.iLimit
    );

    public ShooterController() {
        this.rpmFilter = new LowPassFilter(RobotConstants.Shooter.RPM_LPF_GAIN);
        pidf.setTolerance(RobotConstants.Shooter.RPM_TOLERANCE);
        pidf.setOutputBounds(0.0, RobotConstants.Shooter.MAX_POWER);
    }

    public void reset() {
        pidf.reset();
        rpmFilter.reset();
    }

    public double calculate(double targetRPM, double currentRPM) {
        return pidf.calculate(targetRPM, rpmFilter.estimate(currentRPM));
    }

    public boolean isReady(double targetRPM, double currentRPM) {
        double currentFiltered = rpmFilter.estimate(currentRPM);
        return Utils.withinTolerance(currentFiltered, targetRPM, RobotConstants.Shooter.RPM_TOLERANCE);
    }
}