package org.firstinspires.ftc.teamcode.friends.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.friends.helpers.LowPassFilter;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.friends.helpers.Utils;

public class ShooterController {
    private final LowPassFilter rpmFilter;
    private final PIDFController pidf;
    private final ElapsedTime readyTimer = new ElapsedTime();

    private double currentPower = 0;
    private double targetRPM = 0;
    private double filteredRPM = 0;
    private boolean isWithinTolerance = false;

    public ShooterController() {
        this.rpmFilter = new LowPassFilter(RobotConstants.Shooter.RPM_LPF_GAIN);
        this.pidf = new PIDFController(
                RobotConstants.Shooter.kP,
                RobotConstants.Shooter.kI,
                RobotConstants.Shooter.kD,
                RobotConstants.Shooter.kS,
                RobotConstants.Shooter.kV,
                0.0,
                0.0,
                RobotConstants.Shooter.iLimit
        );
        pidf.setTolerance(RobotConstants.Shooter.RPM_TOLERANCE);
        pidf.setOutputBounds(0.0, RobotConstants.Shooter.MAX_POWER);
        readyTimer.reset();
    }

    public double update(double targetRPM, double currentRPM) {
        this.targetRPM = targetRPM;
        this.filteredRPM = rpmFilter.estimate(currentRPM);

        if (targetRPM <= 10) {
            currentPower = 0;
            pidf.reset();
            isWithinTolerance = false;
        } else {
            currentPower = pidf.calculate(targetRPM, filteredRPM, targetRPM, 0);

            boolean nowInside = Utils.withinTolerance(filteredRPM, targetRPM, RobotConstants.Shooter.RPM_TOLERANCE);

            if (!nowInside) {
                readyTimer.reset();
            }
            isWithinTolerance = nowInside;
        }

        return currentPower;
    }

    /**
     * Returns true only if the shooter has been within tolerance
     * consistently for at least 0.2 seconds.
     */
    public boolean isReady() {
        return isWithinTolerance && readyTimer.seconds() >= 0.2;
    }

    public double getCurrentPower() { return currentPower; }
    public double getFilteredRPM() { return filteredRPM; }
    public double getTargetRPM() { return targetRPM; }

    public void reset() {
        pidf.reset();
        rpmFilter.reset();
        readyTimer.reset();
        currentPower = 0;
        isWithinTolerance = false;
    }

    public void updateConstants() {
        pidf.setPIDF(
                RobotConstants.Shooter.kP,
                RobotConstants.Shooter.kI,
                RobotConstants.Shooter.kD,
                RobotConstants.Shooter.kS,
                RobotConstants.Shooter.kV,
                0.0,
                0.0
        );
        pidf.setTolerance(RobotConstants.Shooter.RPM_TOLERANCE);
    }
}