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
    double MIN_TURRET_ANGLE = -85.0;
    double MAX_TURRET_ANGLE =  85.0;
    int rightTicks = 193;
    int leftTicks = -193;

    // encoder conversion
     double TICKS_PER_DEGREE = (rightTicks - leftTicks) / 180.0;   // need to calibrate
    //TICKS_PER_DEGREE = (rightTicks - leftTicks) / (angleRangeDegrees);

    double currentTurretAngle = 0;


    //--Reverse Motor--

    double lastXError = 0;
    double turretDirection = 1;   // +1 or -1


    /* -------- Constants -------- */


    double kP_rotate = 0.7; // scales x-error → strafe power; higher = faster, lower = smoother

    double kP_drive  = 3.5;

    double MAX_DRIVE_POWER  = 0.6;
    double MAX_ROTATE_TURRET_POWER = 0.15;

    double ROTATE_TOLERANCE = 0.2; //Allows there to be some error

    double DRIVE_TOLERANCE  = 0.2;
    //Desired Shooting Distance is 60 inches at camera


    double INITIAL_SEARCH_TIME = 0.35;  // how long to continue last direction
    double searchPower = 0.15;
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

        currentTurretAngle = (turretEncoderTicks) / TICKS_PER_DEGREE;

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
            if (lostTimer.seconds() > LOST_DELAY) {
                if (currentState != State.SEARCH) {
                    currentState = State.SEARCH;
                    searchTimer.reset();
                }
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
                double targetArea = results.getTa();

                double desiredArea = 1.14; // % of tag occupied at 60 inches

                double areaError = desiredArea - targetArea;

                if (Math.abs(areaError) > DRIVE_TOLERANCE) {
                    drivePower = Range.clip(areaError * kP_drive, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
                }
                if (Math.abs(areaError) < 0.2) {
                    drivePower = 0;
                }
                if (results == null || !results.isValid()) {
                    drivePower = 0;
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

                    // Second phase: sweep between limits
                    turretRotatePower = searchPower * turretDirection;

                   // Flip direction if we hit limits
                    if (currentTurretAngle >= MAX_TURRET_ANGLE && turretDirection > 0) {
                        turretDirection = -1;
                    } else if (currentTurretAngle <= MIN_TURRET_ANGLE && turretDirection < 0) {
                        turretDirection = 1;
                    }

                    // Apply soft limits to prevent overshoot
                    turretRotatePower = Range.clip(
                            turretRotatePower,
                            (currentTurretAngle <= MIN_TURRET_ANGLE) ? 0 : -MAX_ROTATE_TURRET_POWER,
                            (currentTurretAngle >= MAX_TURRET_ANGLE) ? 0 : MAX_ROTATE_TURRET_POWER
                    );
                }

                break;
        }

        // ---------------- SOFT LIMIT PROTECTION ----------------

        if (currentTurretAngle >= MAX_TURRET_ANGLE && turretRotatePower > 0 ) {
            turretRotatePower = 0;
        }

        if (currentTurretAngle <= MIN_TURRET_ANGLE && turretRotatePower < 0 ) {
            turretRotatePower = 0;
        }
    }

}
