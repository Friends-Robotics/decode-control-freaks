package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;


public class VisionAlign {


    public double turretPower = 0;
    public double drivePowerClose = 0;
    public double drivePowerFar = 0;
    public double turretRotatePower = 0;

    double MIN_TURRET_ANGLE = -85.0;
    double MAX_TURRET_ANGLE = 85.0;

    public static class TurretConstants {
        public static final double TICKS_PER_DEGREE = (193 - (-193)) / 180.0;
    }
    double currentTurretAngle = 0;

    public double lastXError = 0;
    public boolean isAligned = false;
    public double alignmentTolerance = 5;
    double turretDirection = 1;

    double kP_rotate = 0.7;
    double kP_drive = 0.8;
    double kP_driveFar = 1;
    double MAX_DRIVE_POWER = 0.6;
    double MAX_ROTATE_TURRET_POWER = 0.10;
    double ROTATE_TOLERANCE = 0.8;
    double DRIVE_TOLERANCE = 0.05;

    double INITIAL_SEARCH_TIME = 1.00;
    double searchPower = 0.15;
    ElapsedTime searchTimer = new ElapsedTime();

    enum State { IDLE, TRACK, SEARCH }
    State currentState = State.IDLE;

    double LOST_DELAY = 0.25;
    ElapsedTime lostTimer = new ElapsedTime();

    public void update(LLResult results, boolean enabled, int turretEncoderTicks) {
        currentTurretAngle = turretEncoderTicks / TurretConstants.TICKS_PER_DEGREE;
        turretRotatePower = 0;
        drivePowerClose = 0;
        isAligned = false;

        if (!enabled) {
            currentState = State.IDLE;
            return;
        }

        boolean tagValid = (results != null && results.isValid());
        if (tagValid) {
            currentState = State.TRACK;
            lostTimer.reset();
        } else if (lostTimer.seconds() > LOST_DELAY) {
            if (currentState != State.SEARCH) {
                currentState = State.SEARCH;
                searchTimer.reset();
            }
        }

        switch (currentState) {
            case IDLE:
                turretRotatePower = 0;
                break;

            case TRACK:
                double xError = results.getTx();
                lastXError = xError;

                if (Math.abs(xError) > ROTATE_TOLERANCE) {
                    turretRotatePower = Range.clip(xError * kP_rotate, -MAX_ROTATE_TURRET_POWER, MAX_ROTATE_TURRET_POWER);
                }
                if(Math.abs(xError) <= alignmentTolerance)
                {
                    isAligned = true;
                }

                double targetArea = results.getTa();
                double desiredAreaClose = 1.14;
                double desiredAreaFar = 0.3162;
                double areaErrorClose = desiredAreaClose - targetArea;
                double areaErrorFar = desiredAreaFar - targetArea;

                drivePowerClose = (Math.abs(areaErrorClose) > DRIVE_TOLERANCE) ? Range.clip(areaErrorClose * kP_drive, -MAX_DRIVE_POWER, MAX_DRIVE_POWER) : 0;
                drivePowerFar   = (Math.abs(areaErrorFar) > DRIVE_TOLERANCE) ? Range.clip(areaErrorFar * kP_driveFar, -MAX_DRIVE_POWER, MAX_DRIVE_POWER) : 0;
                break;

            case SEARCH:
                if (searchTimer.seconds() < INITIAL_SEARCH_TIME) {
                    turretDirection = Math.signum(lastXError);
                    if (turretDirection == 0) turretDirection = 1;
                    turretRotatePower = searchPower * turretDirection;
                } else {
                    turretRotatePower = searchPower * turretDirection;
                    if (currentTurretAngle >= MAX_TURRET_ANGLE && turretDirection > 0) turretDirection = -1;
                    else if (currentTurretAngle <= MIN_TURRET_ANGLE && turretDirection < 0) turretDirection = 1;
                    turretRotatePower = Range.clip(turretRotatePower,
                            (currentTurretAngle <= MIN_TURRET_ANGLE) ? 0 : -MAX_ROTATE_TURRET_POWER,
                            (currentTurretAngle >= MAX_TURRET_ANGLE) ? 0 : MAX_ROTATE_TURRET_POWER);
                }
                break;
        }

        // Soft limits
        if (currentTurretAngle >= MAX_TURRET_ANGLE && turretRotatePower > 0) turretRotatePower = 0;
        if (currentTurretAngle <= MIN_TURRET_ANGLE && turretRotatePower < 0) turretRotatePower = 0;
    }
}
