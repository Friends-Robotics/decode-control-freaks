package org.firstinspires.ftc.teamcode.friends.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;

@Config
public class VisionAlign {
    public static double kP = 0.025;
    public static double kI = 0.005;
    public static double kD = 0.0012;
    public static double kS = 0.05;      // The resistance to turning
    public static double kV = 0.0;       // Resistance is not proportional to velocity so 0
    public static double iLimit = 0.5;

    private final PIDFController turretPID = new PIDFController(kP, kI, kD, kS, kV, iLimit);

    public enum State { IDLE, TRACK, SEARCH }
    private State currentState = State.IDLE;

    public double turretRotatePower = 0;
    public boolean isAligned = false;

    public final double ALIGN_TOLERANCE = 1.5;
    public final double MAX_POWER = 0.7;
    public final double MIN_ANGLE = -42.5;
    public final double MAX_ANGLE = 42.5;

    private final ElapsedTime searchTimer = new ElapsedTime();
    private double searchDirection = 1.0;

    public void update(LLResult results, boolean enabled, int turretTicks, double ticksPerDegree) {
        double currentAngle = turretTicks / ticksPerDegree;

        if (!enabled) {
            stop();
            return;
        }

        boolean tagValid = (results != null && results.isValid());

        if (tagValid) {
            currentState = State.TRACK;
        } else if (currentState == State.TRACK) {
            currentState = State.SEARCH;
            searchTimer.reset();
            turretPID.reset();
        }

        switch (currentState) {
            case TRACK:
                runTrackLogic(results.getTx());
                break;
            case SEARCH:
                runSearchLogic(currentAngle);
                break;
            case IDLE:
                stop();
                break;
        }

        if (currentAngle >= MAX_ANGLE && turretRotatePower > 0) turretRotatePower = 0;
        if (currentAngle <= MIN_ANGLE && turretRotatePower < 0) turretRotatePower = 0;
    }

    private void runTrackLogic(double tx) {
        turretRotatePower = turretPID.calculate(0, -tx);

        if (Math.abs(tx) <= ALIGN_TOLERANCE) {
            isAligned = true;
            turretRotatePower = 0; // Cut power when aligned
        } else {
            isAligned = false;
        }

        turretRotatePower = Range.clip(turretRotatePower, -MAX_POWER, MAX_POWER);
    }

    private void runSearchLogic(double currentAngle) {
        isAligned = false;
        if (currentAngle >= MAX_ANGLE - 2) searchDirection = -1.0;
        if (currentAngle <= MIN_ANGLE + 2) searchDirection = 1.0;

        double centerBias = -currentAngle * 0.01;
        turretRotatePower = (0.3 * searchDirection) + centerBias;
    }

    public void stop() {
        currentState = State.IDLE;
        turretRotatePower = 0;
        turretPID.reset();
        isAligned = false;
    }
}