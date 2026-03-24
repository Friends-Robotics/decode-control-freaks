package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;

public class VisionAlign {

    // PID constants (you will tune these)

    /*Then:
    Increase kP until it tracks fast but starts to oscillate
    Add kD to remove oscillation
    Add a tiny kI only if it never fully centers
    */
    double kP = 0.02;
    double kI = 0.0;
    double kD = 0.002;

    // PID state
    double integralSum = 0;
    double lastError = 0;

    // Prevent integral windup
    double MAX_INTEGRAL = 50;

    // Timer for delta time
    ElapsedTime pidTimer = new ElapsedTime();

    public double turretPower = 0;
    public double drivePowerClose = 0;
    public double drivePowerFar = 0;
    public double turretRotatePower = 0;

    public double lastXError = 0;
    public boolean isAligned = false;

    // alignment tolerance in degrees
    public double alignmentTolerance = 3;

    // turret limits and constants
    double MIN_TURRET_ANGLE = -85.0;
    double MAX_TURRET_ANGLE = 85.0;

    int rightTicks = 193;
    int leftTicks = -193;

    double TICKS_PER_DEGREE = (rightTicks - leftTicks) / 180.0;
    double currentTurretAngle = 0;

    double turretDirection = 1;
    double kP_rotate = 0.25;
    double MAX_ROTATE_TURRET_POWER = 0.6;

    double ROTATE_TOLERANCE = 0.8;
    double DRIVE_TOLERANCE = 0.05;

    double INITIAL_SEARCH_TIME = 0.35;
    double searchPower = 0.4;

    ElapsedTime searchTimer = new ElapsedTime();
    ElapsedTime lostTimer = new ElapsedTime();

    enum State { IDLE, TRACK, SEARCH }
    State currentState = State.IDLE;

    double LOST_DELAY = 0.25;

    public void update(LLResult results, boolean enabled, int turretEncoderTicks) {

        // Prevent divide-by-zero just in case
        if (TICKS_PER_DEGREE == 0) {
            TICKS_PER_DEGREE = 1;
        }

        currentTurretAngle = turretEncoderTicks / TICKS_PER_DEGREE;

        turretRotatePower = 0;
        drivePowerClose = 0;
        isAligned = false;

        if (!enabled) {
            currentState = State.IDLE;

            integralSum = 0;
            lastError = 0;

            return;
        }

        boolean tagValid = (results != null && results.isValid());

        // ---------------- STATE TRANSITIONS ----------------
        if (tagValid) {
            currentState = State.TRACK;
            lostTimer.reset();
        } else if (lostTimer.seconds() > LOST_DELAY) {
            if (currentState != State.SEARCH) {
                currentState = State.SEARCH;
                searchTimer.reset();

                integralSum = 0;
                lastError = 0;
            }
        }

        // ---------------- STATE MACHINE ----------------
        switch (currentState) {

            case IDLE:
                turretRotatePower = 0;
                break;

            case TRACK:



                double xError = results.getTx();
                lastXError = xError;

                double dt = pidTimer.seconds();
                pidTimer.reset();

                // Prevent divide-by-zero
                if (dt == 0) dt = 0.01;

                // ----- PID CALCULATIONS -----
                // Integral (with clamp)
                integralSum += xError * dt;
                integralSum = Range.clip(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);

                // Derivative
                double derivative = (xError - lastError) / dt;

                // PID output
                double output = (kP * xError) + (kI * integralSum) + (kD * derivative);

                lastError = xError;

                // Apply power with limits
                turretRotatePower = Range.clip(output, -MAX_ROTATE_TURRET_POWER, MAX_ROTATE_TURRET_POWER);

                // Alignment check
                if (Math.abs(xError) < alignmentTolerance) {
                    isAligned = true;

                    // Reset integral when aligned (prevents drift)
                    integralSum = 0;
                }

                if (currentTurretAngle >= MAX_TURRET_ANGLE && turretPower > 0)
                    turretPower = 0;

                if (currentTurretAngle <= MIN_TURRET_ANGLE && turretPower < 0)
                    turretPower = 0;

                break;

            case SEARCH:

                // If we never had a last error, default direction
                if (lastXError == 0) {
                    turretDirection = 1;
                }

                if (searchTimer.seconds() < INITIAL_SEARCH_TIME) {
                    turretDirection = Math.signum(lastXError);
                    if (turretDirection == 0) turretDirection = 1;
                }

                turretRotatePower = searchPower * turretDirection;

                // Bounce off limits
                if (currentTurretAngle >= MAX_TURRET_ANGLE && turretDirection > 0) {
                    turretDirection = -1;
                } else if (currentTurretAngle <= MIN_TURRET_ANGLE && turretDirection < 0) {
                    turretDirection = 1;
                }

                // Soft limits (prevents slamming)
                if (currentTurretAngle > MAX_TURRET_ANGLE - 5) {
                    turretDirection = -1;
                }
                if (currentTurretAngle < MIN_TURRET_ANGLE + 5) {
                    turretDirection = 1;
                }

                // Stop pushing INTO the limit, but allow movement AWAY
                if (currentTurretAngle >= MAX_TURRET_ANGLE && turretPower > 0)
                    turretPower = 0;

                if (currentTurretAngle <= MIN_TURRET_ANGLE && turretPower < 0)
                    turretPower = 0;


                break;
        }
    }
}