package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.friends.components.Turret;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;

/**
 * VisionAlign handles the transition between manual driver control (Home)
 * and automated Limelight targeting (Track).
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
    public boolean isAligned = false;
    public boolean hasTarget = false;

    /**
     * Main update loop for turret vision alignment.
     * @param results Current Limelight results
     * @param enabled Whether the driver is holding the "Aim Assist" button
     * @param currentAngle Current turret angle from encoders
     */
    public void update(LLResult results, boolean enabled, double currentAngle) {
        // Check Limelight status
        hasTarget = (results != null && results.isValid());

        // State Logic: If enabled AND we see a target, TRACK. Otherwise, HOME.
        if (enabled && hasTarget) {
            currentState = State.TRACK;
        } else {
            // Reset integral when switching out of track to prevent windup
            if (currentState == State.TRACK) {
                turretPID.reset();
            }
            currentState = State.HOME;
        }

        // Execute logic based on state
        switch (currentState) {
            case TRACK:
                runTrackLogic(results.getTx());
                break;
            case HOME:
                runHomeLogic(currentAngle);
                break;
        }

        // Safety: Hard-stop power if we are hitting physical limits
        applySafetyClamps(currentAngle);
    }

    /**
     * Aligns the turret crosshair (tx) to 0.
     */
    private void runTrackLogic(double tx) {
        // tx is the horizontal offset. We want to minimize this to 0.
        // We use -tx because if tx is positive (target is right),
        // we need to move the turret in a way that brings tx back to center.
        turretRotatePower = turretPID.calculate(0, -tx);

        isAligned = Math.abs(tx) <= RobotConstants.Turret.ALIGN_TOLERANCE;

        // Clip to max allowed speed for tracking
        turretRotatePower = Range.clip(turretRotatePower,
                -RobotConstants.Turret.MAX_POWER,
                RobotConstants.Turret.MAX_POWER);
    }

    /**
     * Returns the turret to the center (0 degrees) relative to the robot.
     */
    private void runHomeLogic(double currentAngle) {
        isAligned = false; // We aren't aligned to a target if we are homing

        // Return to 0 degrees
        turretRotatePower = turretPID.calculate(0, currentAngle);

        // Clip to a gentler speed for homing so it doesn't snap back violently
        turretRotatePower = Range.clip(turretRotatePower,
                -RobotConstants.Turret.HOME_POWER,
                RobotConstants.Turret.HOME_POWER);

        // Deadband to stop motor chatter at home
        if (Math.abs(currentAngle) <= RobotConstants.Turret.ALIGN_TOLERANCE) {
            turretRotatePower = 0;
        }
    }

    /**
     * Prevents the PID from trying to drive the turret past physical stops.
     */
    private void applySafetyClamps(double currentAngle) {
        if (currentAngle >= Turret.MAX_ANGLE && turretRotatePower > 0) {
            turretRotatePower = 0;
        }
        if (currentAngle <= Turret.MIN_ANGLE && turretRotatePower < 0) {
            turretRotatePower = 0;
        }
    }

    public double getOutputPower() { return turretRotatePower; }
    public State getCurrentState() { return currentState; }
}