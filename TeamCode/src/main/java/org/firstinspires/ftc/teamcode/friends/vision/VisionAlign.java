package org.firstinspires.ftc.teamcode.friends.vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionAlign {

    // PID constants need tuned

    /*Then:
    Increase kP until it tracks fast but starts to oscillate
    Add kD to remove oscillation
    Add a tiny kI only if it never fully centers
    */

    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.002;
    public static double kF = 0.05;

    // PID state
    double integralSum = 0;
    double lastError = 0;

    // Prevent integral windup
    double MAX_INTEGRAL = 50;

    // Timer for delta time
    ElapsedTime pidTimer = new ElapsedTime();

    public double drivePowerClose = 0;
    public double drivePowerFar = 0;
    public double turretRotatePower = 0;

    public double lastXError = 0;

    public boolean isAligned = false;

    // alignment tolerance in degrees
    public double alignmentTolerance = 3;

    // turret limits and constants
    double MIN_TURRET_ANGLE = -42.5;
    double MAX_TURRET_ANGLE = 42.5;

    int rightTicks = 193;
    int leftTicks = -193;

    double TICKS_PER_DEGREE = (rightTicks - leftTicks) / 180.0;
    public double currentTurretAngle = 0;

    double turretDirection = 1;

    double MAX_ROTATE_TURRET_POWER = 0.6;

    //SEARCH
    double lastKnownTargetAngle = 0;
    boolean hasSeenTarget = false;
    double INITIAL_SEARCH_TIME = 0.35;
    double ALIGNED_TIME = 0.4;
    double searchPower = 0.4;

    ElapsedTime searchTimer = new ElapsedTime();
    ElapsedTime lostTimer = new ElapsedTime();
    ElapsedTime alignedTimer = new ElapsedTime();

    enum State { IDLE, TRACK, SEARCH }
    State currentState = State.IDLE;
    State lastState = currentState;

    double LOST_DELAY = 0.25;



    public void update(LLResult results, boolean enabled, int turretEncoderTicks, Pose currentPose, Pose goalPose) {

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
        if (currentState == State.TRACK && lastState != State.TRACK) {
            pidTimer.reset();
            alignedTimer.reset();
        }

        // ---------------- STATE MACHINE ----------------
        switch (currentState) {

            case IDLE:
                turretRotatePower = 0;
                break;

            case TRACK:

                double alpha = 0.85;
                double xError = alpha * results.getTx() + (1 - alpha) * lastXError;

                double dt = pidTimer.seconds();
                pidTimer.reset();
                if (dt == 0) dt = 0.01;

                // I
                integralSum += xError * dt;
                integralSum = Range.clip(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);

                // D
                double derivative = (xError - lastError) / dt;

                // PID
                double output = (kP * xError) + (kI * integralSum) + (kD * derivative);

                // Feedforward
                output += kF * (xError / 27.0);

                turretRotatePower = Range.clip(output, -MAX_ROTATE_TURRET_POWER, MAX_ROTATE_TURRET_POWER);

                // Alignment logic
                if (Math.abs(xError) < alignmentTolerance) {
                    if (alignedTimer.seconds() > 0.2) {
                        isAligned = true;
                    }
                } else {
                    alignedTimer.reset();
                    isAligned = false;
                }

                // Limits
                if (currentTurretAngle >= MAX_TURRET_ANGLE && turretRotatePower > 0)
                    turretRotatePower = 0;
                if (currentTurretAngle <= MIN_TURRET_ANGLE && turretRotatePower < 0)
                    turretRotatePower = 0;

                lastKnownTargetAngle = currentTurretAngle;
                hasSeenTarget = true;
                lastXError = xError;
                lastError = xError;

                break;

            case SEARCH:
                turretRotatePower = searchPower * turretDirection;

                if (currentTurretAngle >= MAX_TURRET_ANGLE) {
                    turretDirection = -1;
                } else if (currentTurretAngle <= MIN_TURRET_ANGLE) {
                    turretDirection = 1;
                }

                break;
        }
    }
}