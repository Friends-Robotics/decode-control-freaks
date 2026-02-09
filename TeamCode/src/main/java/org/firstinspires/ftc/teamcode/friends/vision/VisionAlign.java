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
    double TICKS_PER_DEGREE = 10.5;  // need to calibrate
    //TICKS_PER_DEGREE = (rightTicks - leftTicks) / (angleRangeDegrees);

    double currentTurretAngle = 0;


    //--Reverse Motor--

    int reverseCount = 0;
    double lastXError = 0;
    int turretDirection = 1;   // +1 or -1
    boolean directionLocked = false;


    /* -------- Constants -------- */

    double scaledKP = 0;
    double kP_rotate = 0.6; // scales x-error → strafe power; higher = faster, lower = smoother
    double kP_turret = 0.015;
    double kP_drive  = 0.7;

    double MAX_TURRET_POWER = 0.35;
    double MAX_DRIVE_POWER  = 0.4;
    double MAX_ROTATE_TURRET_POWER = 0.25;

    double ROTATE_TOLERANCE = 0.03; //Allows there to be some error
    double TURRET_TOLERANCE = 1.0;
    double DRIVE_TOLERANCE  = 0.05;

    double DESIRED_DISTANCE_METERS = 70 * 0.0254;
    double VISION_TIMEOUT = 0.4; //Vision doesnt compromise everything if it fails
    boolean timerStarted = false;

    ElapsedTime timer = new ElapsedTime();

    /*--- MAIN METHOD --- */

    public void update(LLResult result, boolean enable, int turretEncoderTicks) {

        currentTurretAngle = turretEncoderTicks / TICKS_PER_DEGREE;
        turretPower = 0;
        drivePower  = 0;
        turretRotatePower = 0;

        if (!enable) {
            timer.reset();   // give vision time to recover
            timerStarted = false;
            directionLocked = false;
            lastXError = 0;
            return;
        }



        if (!timerStarted) {
            timer.reset();
            timerStarted = true;
        }

        if (timer.seconds() > VISION_TIMEOUT) return;


        if (result == null || !result.isValid()) {
            directionLocked = false;
            lastXError = 0;
            return;
        }


        Pose3D pose = result.getBotpose();

        /* ---------- Rotating Turret ---------- */
        double xError = pose.getPosition().x;

       // Auto-detect motor direction (flip once if error worsens)

        if (!directionLocked && Math.abs(lastXError) > ROTATE_TOLERANCE) {
            if (Math.abs(xError) > Math.abs(lastXError)) {
                reverseCount++;
                if (reverseCount > 3) {
                    turretDirection *= -1;
                    directionLocked = true;
                }
            } else reverseCount = 0;
        }


        // Proportional control with direction applied
        double minPower = 0.06;

        if (Math.abs(xError) > ROTATE_TOLERANCE) {

            scaledKP = Range.clip(
                    Math.abs(xError) / 0.25,// at 25cm use 100% of kp power
                    0.2,
                    1.0
            );

            turretRotatePower = Math.signum(xError) * turretDirection *
                    (minPower + Math.abs(xError * kP_rotate));

            turretRotatePower = Range.clip(
                    turretRotatePower,
                    -MAX_ROTATE_TURRET_POWER,
                    MAX_ROTATE_TURRET_POWER
            );
        }

        /* ---------- Software Angle Limit ---------- */

        // Trying to rotate past max
        if (currentTurretAngle >= MAX_TURRET_ANGLE && turretRotatePower > 0) {
            turretRotatePower = 0;
        }

        // Trying to rotate past min
        if (currentTurretAngle <= MIN_TURRET_ANGLE && turretRotatePower < 0) {
            turretRotatePower = 0;
        }
        //Slows down as it get close to limit
        double limitBuffer = 10.0; // degrees

        if (currentTurretAngle > MAX_TURRET_ANGLE - limitBuffer) {
            double scale = (MAX_TURRET_ANGLE - currentTurretAngle) / limitBuffer;
            turretRotatePower *= Range.clip(scale, 0.0, 1.0);
        }

        if (currentTurretAngle < MIN_TURRET_ANGLE + limitBuffer) {
            double scale = (currentTurretAngle - MIN_TURRET_ANGLE) / limitBuffer;
            turretRotatePower *= Range.clip(scale, 0.0, 1.0);
        }

        lastXError = xError;

        /* ---------- TURRET ---------- */
        /*double yawError = -pose.getOrientation().getYaw(); //Turret yaw is opposite to camera yaw
        if (Math.abs(yawError) > TURRET_TOLERANCE) {
            turretPower = Range.clip(
                    yawError * kP_turret,
                    -MAX_TURRET_POWER,
                    MAX_TURRET_POWER
            );
        }
        */
        /* ---------- DISTANCE ---------- */
        double distanceError =
                pose.getPosition().z - DESIRED_DISTANCE_METERS;

        if (Math.abs(distanceError) > DRIVE_TOLERANCE) {
            drivePower = Range.clip(
                    distanceError * kP_drive,
                    -MAX_DRIVE_POWER,
                    MAX_DRIVE_POWER
            );
        }
    }
}
