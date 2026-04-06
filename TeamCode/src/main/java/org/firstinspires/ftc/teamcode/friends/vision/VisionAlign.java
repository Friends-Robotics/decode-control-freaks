package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;

public class VisionAlign {
    private final PIDFController turretPID = new PIDFController(
            RobotConstants.Vision.kP,
            RobotConstants.Vision.kI,
            RobotConstants.Vision.kD,
            RobotConstants.Vision.kS,
            RobotConstants.Vision.kV,
            RobotConstants.Vision.iLimit
    );

    public enum State { TRACK, SEARCH, HOME }
    private State currentState = State.HOME;

    private double turretRotatePower = 0;
    public boolean isAligned = false;

    private final ElapsedTime searchTimer = new ElapsedTime();
    private double searchDirection = 1.0;

    public void update(LLResult results, boolean enabled, double currentAngle) {
        // Master switch
        if (!enabled) {
            currentState = State.HOME;
        } else {
            if (results != null && results.isValid()) {
                currentState = State.TRACK;
            } else {
                if (currentState != State.SEARCH) {
                    searchTimer.reset();
                    turretPID.reset();
                }
                currentState = State.SEARCH;
            }
        }

        // Logic Switch
        switch (currentState) {
            case TRACK:  runTrackLogic(results.getTx()); break;
            case SEARCH: runSearchLogic(currentAngle); break;
            case HOME:   runHomeLogic(currentAngle); break;
        }

        // Safety Clamps using Constants
        if (currentAngle >= RobotConstants.Vision.MAX_ANGLE && turretRotatePower > 0) turretRotatePower = 0;
        if (currentAngle <= RobotConstants.Vision.MIN_ANGLE && turretRotatePower < 0) turretRotatePower = 0;
    }

    private void runTrackLogic(double tx) {
        turretRotatePower = turretPID.calculate(0, -tx);
        isAligned = Math.abs(tx) <= RobotConstants.Vision.ALIGN_TOLERANCE;
        if (isAligned) turretRotatePower = 0;
        turretRotatePower = Range.clip(turretRotatePower, -RobotConstants.Vision.MAX_POWER, RobotConstants.Vision.MAX_POWER);
    }

    private void runSearchLogic(double angle) {
        // Use a small buffer (2 degrees) so it doesn't get stuck on the "Zero Power" clamp
        if (angle >= RobotConstants.Vision.MAX_ANGLE - 2) searchDirection = -1.0;
        if (angle <= RobotConstants.Vision.MIN_ANGLE + 2) searchDirection = 1.0;

        double sweep = 0.35 * searchDirection;
        double bias = -angle * 0.01; // Soft pull toward center
        turretRotatePower = sweep + bias;
    }

    private void runHomeLogic(double angle) {
        turretRotatePower = turretPID.calculate(0, angle);
        turretRotatePower = Range.clip(turretRotatePower, -RobotConstants.Vision.HOME_POWER, RobotConstants.Vision.HOME_POWER);
        if (Math.abs(angle) <= RobotConstants.Vision.ALIGN_TOLERANCE) turretRotatePower = 0;
    }

    public double getOutputPower() { return turretRotatePower; }
}