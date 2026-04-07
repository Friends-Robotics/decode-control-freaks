package org.firstinspires.ftc.teamcode.friends.controllers;

import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;

public class ShooterController {
    private final PIDFController shooterPIDF = new PIDFController(
            RobotConstants.Shooter.kP,
            RobotConstants.Shooter.kI,
            RobotConstants.Shooter.kD,
            RobotConstants.Shooter.kS,
            RobotConstants.Shooter.kV,
            RobotConstants.Shooter.iLimit
    );

    private boolean isShooting = false; // Latch for continuous fire

    /**
     * @param robot Access to shooter/intake motors
     * @param shootButtonPressed Driver input (gamepad2.right_trigger > 0.5)
     * @param targetRPM Desired velocity (manual or distance-calculated)
     */
    public void update(Robot robot, boolean shootButtonPressed, double targetRPM, double hoodPosition) {
        double currentRPM = robot.getShooterRPM();

        double currentTarget = shootButtonPressed ? targetRPM : RobotConstants.Shooter.IDLE_RPM;

        double shooterPower = shooterPIDF.calculate(currentTarget, currentRPM);
        robot.setShooterPower(shooterPower);

        robot.setHoodPosition(hoodPosition);

        if (shootButtonPressed) {
            robot.startFeed();

            boolean atSpeed = Math.abs(targetRPM - currentRPM) < RobotConstants.Shooter.RPM_TOLERANCE;

            if (atSpeed) {
                isShooting = true;
            }
        } else {
            isShooting = false;
            robot.stopFeed();
        }

        if (isShooting) {
            robot.intake();
        } else {
            robot.stopIntake();
        }
    }

    public boolean isShooting() { return isShooting; }
}