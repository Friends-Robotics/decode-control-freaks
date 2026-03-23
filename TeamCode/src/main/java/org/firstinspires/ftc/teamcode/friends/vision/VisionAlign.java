package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * Vision-driven turret controller.
 * Vision (tx) gives the base angle to the tag.
 * OdometryShooter gives a small offset to aim behind the tag (toward goal).
 * Turret tracks the combined target angle.
 */
public class VisionAlign {

    public double drivePowerClose = 0;
    public double drivePowerFar = 0;
    public double turretRotatePower = 0;

    public double lastXError = 0;
    public boolean isAligned = false;

    // Alignment tolerance in degrees of turret error
    public double alignmentTolerance = 1.5;

    // Turret limits and constants
    double MIN_TURRET_ANGLE = -85.0;
    double MAX_TURRET_ANGLE = 85.0;
    int rightTicks = 193;
    int leftTicks = -193;
    double TICKS_PER_DEGREE = (rightTicks - leftTicks) / 180.0;
    public double currentTurretAngle = 0;

    double turretDirection = 1;
    double kP_rotate = 0.02;   // start smaller, tune up
    double kP_drive = 0.8;
    double kP_driveFar = 1;
    double MAX_DRIVE_POWER = 0.6;
    double MAX_ROTATE_TURRET_POWER = 0.12;
    double DRIVE_TOLERANCE = 0.05;

    double INITIAL_SEARCH_TIME = 0.35;
    double searchPower = 0.15;
    ElapsedTime searchTimer = new ElapsedTime();

    enum State { IDLE, TRACK, SEARCH }
    State currentState = State.IDLE;

    double LOST_DELAY = 0.25;
    ElapsedTime lostTimer = new ElapsedTime();

    public void update(LLResult results,
                       boolean enabled,
                       int turretEncoderTicks,
                       double odometryOffsetDegrees) {

        // Convert ticks → degrees
        currentTurretAngle = turretEncoderTicks / TICKS_PER_DEGREE;

        turretRotatePower = 0;
        drivePowerClose = 0;
        drivePowerFar = 0;
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
                // Use lastXError if tag flickers
                if (tagValid && Math.abs(results.getTx()) < 50) {
                    lastXError = 0.8 * lastXError + 0.2 * results.getTx();
                }

                double xError = lastXError;

                // Vision angle: angle from turret to tag (in degrees)
                double visionAngle = -xError; // sign depends on your setup

                // Combined target angle: tag + odometry offset
                double finalTargetAngle = visionAngle + odometryOffsetDegrees;

                // Turret error = where we want to be - where we are
                double alignError = finalTargetAngle - currentTurretAngle;

                if (Math.abs(alignError) > alignmentTolerance) {
                    turretRotatePower = Range.clip(
                            alignError * kP_rotate,
                            -MAX_ROTATE_TURRET_POWER,
                            MAX_ROTATE_TURRET_POWER
                    );
                } else {
                    isAligned = true;
                    turretRotatePower = 0;
                }

                // Distance control (unchanged from your logic)
                if (tagValid) {
                    double targetArea = results.getTa();
                    double desiredAreaClose = 1.14;
                    double desiredAreaFar = 0.3162;
                    double areaErrorClose = desiredAreaClose - targetArea;
                    double areaErrorFar = desiredAreaFar - targetArea;

                    drivePowerClose = (Math.abs(areaErrorClose) > DRIVE_TOLERANCE)
                            ? Range.clip(areaErrorClose * kP_drive, -MAX_DRIVE_POWER, MAX_DRIVE_POWER)
                            : 0;

                    drivePowerFar = (Math.abs(areaErrorFar) > DRIVE_TOLERANCE)
                            ? Range.clip(areaErrorFar * kP_driveFar, -MAX_DRIVE_POWER, MAX_DRIVE_POWER)
                            : 0;
                }
                break;

            case SEARCH:
                if (searchTimer.seconds() < INITIAL_SEARCH_TIME) {
                    turretDirection = Math.signum(lastXError);
                    if (turretDirection == 0) turretDirection = 1;
                    turretRotatePower = searchPower * turretDirection;
                } else {
                    turretRotatePower = searchPower * turretDirection;

                    if (currentTurretAngle >= MAX_TURRET_ANGLE && turretDirection > 0)
                        turretDirection = -1;
                    else if (currentTurretAngle <= MIN_TURRET_ANGLE && turretDirection < 0)
                        turretDirection = 1;

                    turretRotatePower = Range.clip(
                            turretRotatePower,
                            (currentTurretAngle <= MIN_TURRET_ANGLE) ? 0 : -MAX_ROTATE_TURRET_POWER,
                            (currentTurretAngle >= MAX_TURRET_ANGLE) ? 0 : MAX_ROTATE_TURRET_POWER
                    );
                }
                break;
        }

        // Soft limits
        if (currentTurretAngle >= MAX_TURRET_ANGLE && turretRotatePower > 0)
            turretRotatePower = 0;

        if (currentTurretAngle <= MIN_TURRET_ANGLE && turretRotatePower < 0)
            turretRotatePower = 0;
    }
}