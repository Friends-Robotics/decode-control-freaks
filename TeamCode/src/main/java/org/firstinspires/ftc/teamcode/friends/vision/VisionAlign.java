package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;



public class VisionAlign {


    /* -------- Outputs -------- */
    public double turretPower = 0;
    public double drivePower  = 0;
    public double turretRotatePower = 0;

    /* ---------- Turret Angle Limits ---------- */

    // degrees
    double MIN_TURRET_ANGLE = -90.0;
    double MAX_TURRET_ANGLE =  90.0;

    // encoder conversion
    double TICKS_PER_DEGREE = 0.389;  // need to calibrate
    //TICKS_PER_DEGREE = (rightTicks - leftTicks) / (angleRangeDegrees);

    double currentTurretAngle = 0;


    //--Reverse Motor--

    double lastXError = 0;
    double turretDirection = 1;   // +1 or -1


    /* -------- Constants -------- */

    double scaledKP = 0;
    double kP_rotate = 0.6; // scales x-error → strafe power; higher = faster, lower = smoother
    double kP_turret = 0.015;
    double kP_drive  = 0.7;

    double MAX_TURRET_POWER = 0.35;
    double MAX_DRIVE_POWER  = 0.4;
    double MAX_ROTATE_TURRET_POWER = 0.5;

    double ROTATE_TOLERANCE = 0.03; //Allows there to be some error
    double TURRET_TOLERANCE = 1.0;
    double DRIVE_TOLERANCE  = 0.05;

    double DESIRED_DISTANCE_METERS = 70 * 0.0254;
    double INITIAL_SEARCH_TIME = 0.35;  // how long to continue last direction
    double searchPower = 0.18;
    ElapsedTime searchTimer = new ElapsedTime();



    // -------- STATE MACHINE ------
    enum State {
        IDLE,
        TRACK,
        SEARCH
    }

    State currentState = State.IDLE;

    // Search settings
    double LOST_DELAY = 0.25; // small delay before starting search
    ElapsedTime lostTimer = new ElapsedTime();

    public void update(LLResult results, boolean enabled, int turretEncoderTicks) {

        currentTurretAngle = turretEncoderTicks / TICKS_PER_DEGREE;

        turretRotatePower = 0;
        drivePower = 0;

        // ---------------- STATE TRANSITIONS ----------------

        if (!enabled) {
            currentState = State.IDLE;
            return;
        }

        boolean tagValid = (results != null && results.isValid());

        if (tagValid) {
            currentState = State.TRACK;
            lostTimer.reset();
        } else {
            if (currentState == State.TRACK) {
                lostTimer.reset();
            }

            if (lostTimer.seconds() > LOST_DELAY) {
                currentState = State.SEARCH;
                searchTimer.reset();
            }
        }

        // ---------------- STATE LOGIC ----------------

        switch (currentState) {

            case IDLE:
                turretRotatePower = 0;
                break;

            case TRACK:

                double xError = results.getTx();
                lastXError = xError;   // store last known direction

                if (Math.abs(xError) > ROTATE_TOLERANCE) {
                    turretRotatePower =
                            Range.clip(
                                    xError * kP_rotate,
                                    -MAX_ROTATE_TURRET_POWER,
                                    MAX_ROTATE_TURRET_POWER
                            );
                }

                // Distance control
                Pose3D pose = results.getBotpose();
                if (pose != null) {
                    double distanceError =
                            pose.getPosition().z - DESIRED_DISTANCE_METERS;

                    if (Math.abs(distanceError) > DRIVE_TOLERANCE) {
                        drivePower =
                                Range.clip(
                                        distanceError * kP_drive,
                                        -MAX_DRIVE_POWER,
                                        MAX_DRIVE_POWER
                                );
                    }
                }

                break;

            case SEARCH:

                // First phase: continue in last seen direction
                if (searchTimer.seconds() < INITIAL_SEARCH_TIME) {

                    turretDirection = Math.signum(lastXError);

                    // If lastXError was 0, default to right
                    if (turretDirection == 0) turretDirection = 1;

                    turretRotatePower = searchPower * turretDirection;

                } else {

                    // Second phase: full sweep between limits
                    turretRotatePower = searchPower * turretDirection;

                    if (currentTurretAngle >= MAX_TURRET_ANGLE) {
                        turretDirection = -1;
                    }

                    if (currentTurretAngle <= MIN_TURRET_ANGLE) {
                        turretDirection = 1;
                    }
                }

                break;
        }

        // ---------------- SOFT LIMIT PROTECTION ----------------

        if (currentTurretAngle >= MAX_TURRET_ANGLE && turretRotatePower > 0) {
            turretRotatePower = 0;
        }

        if (currentTurretAngle <= MIN_TURRET_ANGLE && turretRotatePower < 0) {
            turretRotatePower = 0;
        }
    }
}
