package org.firstinspires.ftc.teamcode.friends.vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.friends.comp.Everything;
import org.firstinspires.ftc.teamcode.friends.comp.Helpers;
import org.firstinspires.ftc.teamcode.friends.tests.OdometryShooter;


public class VisionAlign {

    // PID constants need tuned

    /*Then:
    Increase kP until it tracks fast but starts to oscillate
    Add kD to remove oscillation
    Add a tiny kI only if it never fully centers
    */
    Helpers help;
    OdometryShooter odometry;
    Everything TeleOp;
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

    enum State { IDLE, TRACK, SEARCH, CORRECT }
    State currentState = State.IDLE;

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
        if (currentState == State.TRACK && isAligned) {
            if (alignedTimer.seconds() == 0) alignedTimer.reset();

            if (alignedTimer.seconds() > ALIGNED_TIME) {
                currentState = State.CORRECT;
            }
        } else {
            alignedTimer.reset();
        }

        // ---------------- STATE MACHINE ----------------
        switch (currentState) {

            case IDLE:
                turretRotatePower = 0;
                break;

            case TRACK:

                double alpha = 0.7; // 0 = smooth, 1 = raw
                double xError = alpha * lastXError + (1 - alpha) * results.getTx();

                double dt = pidTimer.seconds();
                pidTimer.reset();
                if (dt == 0) dt = 0.01;

                //I
                integralSum += xError * dt;
                integralSum = Range.clip(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);

                //D
                double derivative = (xError - lastError) / dt;

                //F
                double kF = 0.05;

                //Compute TurretPower
                double output = (kP * xError) + (kI * integralSum) + (kD * derivative);

                if (Math.abs(xError) > alignmentTolerance) {
                    output += Math.signum(xError) * kF;
                }

                turretRotatePower = Range.clip(output, -MAX_ROTATE_TURRET_POWER, MAX_ROTATE_TURRET_POWER);

                if (Math.abs(xError) < alignmentTolerance) {
                    isAligned = true;
                    integralSum = 0;
                }

                // Hard clamps — once, at the end
                if (currentTurretAngle >= MAX_TURRET_ANGLE && turretRotatePower > 0)
                    turretRotatePower = 0;
                if (currentTurretAngle <= MIN_TURRET_ANGLE && turretRotatePower < 0)
                    turretRotatePower = 0;

                lastKnownTargetAngle = currentTurretAngle;
                hasSeenTarget = true;
                lastXError = xError;

                break;

            case CORRECT:
                double movement = Math.hypot(help.drive, help.strafe) + Math.abs(help.rotate);
                boolean isStationary = movement < 0.05;

                if (!isStationary) {
                    // if robot moves, immediately go back to TRACK
                    currentState = State.TRACK;
                    break;
                }

                // -------------------------
                // APPLY ODOMETRY CORRECTION
                // -------------------------

                if (results != null && results.isValid()) {

                    double correction = odometry.VisionShooterCorrection(
                            currentPose,
                            goalPose
                    );

                    turretRotatePower = Range.clip(
                            correction,
                            -MAX_ROTATE_TURRET_POWER,
                            MAX_ROTATE_TURRET_POWER
                    );

                } else {
                    currentState = State.SEARCH;
                }

                break;

            case SEARCH:
                double time = searchTimer.seconds();

                // -------------------------
                // PHASE 1: Look where target was last seen
                // -------------------------
                if (time < 0.5 && hasSeenTarget) {

                    double error = lastKnownTargetAngle - currentTurretAngle;

                    turretRotatePower = Range.clip(
                            error * 0.02,
                            -0.4,
                            0.4
                    );
                }

                // -------------------------
                // PHASE 2: Small sweep (center-focused)
                // -------------------------
                else if (time < 1.5) {

                    double centerBias = -currentTurretAngle * 0.01;

                    turretRotatePower = (searchPower * 0.5 * turretDirection) + centerBias;
                }

                // -------------------------
                // PHASE 3: Full sweep
                // -------------------------
                else {

                    turretRotatePower = searchPower * turretDirection;
                }

                // -------------------------
                // EDGE HANDLING (bounce)
                // -------------------------
                if (currentTurretAngle >= MAX_TURRET_ANGLE) {
                    turretDirection = -1;
                } else if (currentTurretAngle <= MIN_TURRET_ANGLE) {
                    turretDirection = 1;
                }

                // -------------------------
                // SLOW DOWN near center (better vision pickup)
                // -------------------------
                if (Math.abs(currentTurretAngle) < 10) {
                    turretRotatePower *= 0.6;
                }

                double bias = -currentTurretAngle * 0.015;
                turretRotatePower += bias;

                break;
        }
    }
}