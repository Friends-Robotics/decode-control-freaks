package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;

/**
 * VisionAlign handles the transition between manual/home control and
 * automated Limelight targeting.
 */
public class VisionAlign {
    private final PIDFController turretPID = new PIDFController(
            RobotConstants.Turret.kP,
            RobotConstants.Turret.kI,
            RobotConstants.Turret.kD,
            RobotConstants.Turret.kS,
            RobotConstants.Turret.kV,
            RobotConstants.Turret.iLimit
    );

    public enum State { TRACK, HOME }
    private State currentState = State.HOME;

    private double turretRotatePower = 0;
    private boolean isAligned = false;
    private boolean hasTarget = false;
    private boolean lastHasTarget = false;
    private boolean newTargetAcquired = false;

    /**
     * Main update loop for turret vision alignment.
     * @param results Current Limelight results
     * @param doTracking Whether the driver is holding the "Aim Assist" button
     * @param currentAngle Current turret angle from encoders
     */
    public void update(LLResult results, boolean doTracking, double currentAngle) {
        hasTarget = (results != null && results.isValid());

        // Set to true for exactly one frame when the target enters sight
        newTargetAcquired = (hasTarget && !lastHasTarget);
        lastHasTarget = hasTarget;

        if (doTracking) {
            currentState = State.TRACK;
        } else {
            if (currentState == State.TRACK) {
                turretPID.reset(); // Reset the PID controller when switching to HOME from TRACK
            }
            currentState = State.HOME;
        }

        switch (currentState) {
            case TRACK:
                if (hasTarget) {
                    runTrackLogic(results.getTx());
                } else {
                    turretRotatePower = 0;
                    isAligned = false;
                }
                break;
            case HOME:
                runHomeLogic(currentAngle);
                break;
        }
    }

    private void runTrackLogic(double tx) {
        isAligned = false;

        // We use -tx because if tx is positive (target right),
        // we need to move the turret to bring it back to center.
        turretRotatePower = turretPID.calculate(0, -tx);

        turretRotatePower = Range.clip(turretRotatePower,
                -RobotConstants.Turret.MAX_POWER,
                RobotConstants.Turret.MAX_POWER);

        // Deadband
        if (Math.abs(tx) <= RobotConstants.Turret.ALIGN_TOLERANCE) {
            turretRotatePower = 0;
            isAligned = true;
        }
    }

    private void runHomeLogic(double currentAngle) {
        isAligned = false;

        turretRotatePower = turretPID.calculate(0, currentAngle);

        turretRotatePower = Range.clip(turretRotatePower,
                -RobotConstants.Turret.HOME_POWER,
                RobotConstants.Turret.HOME_POWER);

        // Deadband
        if (Math.abs(currentAngle) <= RobotConstants.Turret.ALIGN_TOLERANCE) {
            turretRotatePower = 0;
        }
    }

    public double getOutputPower() {
        return turretRotatePower;
    }

    public State getCurrentState() {
        return currentState;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public boolean isAligned() {
        return isAligned;
    }

    public boolean isNewTargetAcquired() {
        return newTargetAcquired;
    }
}