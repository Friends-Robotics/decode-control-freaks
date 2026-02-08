package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;



public class VisionAlign {

    /* -------- Outputs -------- */
    public double turretPower = 0;
    public double drivePower  = 0;
    public double strafePower = 0;

    /* -------- Constants -------- */
    double kP_strafe = 0.6; // scales x-error → strafe power; higher = faster, lower = smoother
    double kP_turret = 0.015;
    double kP_drive  = 0.7;

    double MAX_TURRET_POWER = 0.35;
    double MAX_DRIVE_POWER  = 0.4;
    double MAX_STRAFE_POWER = 0.25;

    double STRAFE_TOLERANCE = 0.03; //Allows there to be some error
    double TURRET_TOLERANCE = 1.0;
    double DRIVE_TOLERANCE  = 0.05;

    double DESIRED_DISTANCE_METERS = 70 * 0.0254;
    double VISION_TIMEOUT = 0.4; //Vision doesnt compromise everything if it fails

    ElapsedTime timer = new ElapsedTime();

    /*--- MAIN METHOD --- */

    public void update(LLResult result, boolean enable) {

        turretPower = 0;
        drivePower  = 0;
        strafePower = 0;

        if (!enable) {
            timer.reset();
            return;
        }

        if (timer.seconds() == 0) timer.reset();
        if (timer.seconds() > VISION_TIMEOUT) return;

        if (result == null || !result.isValid()) return;

        Pose3D pose = result.getBotpose();

        /* ---------- STRAFE ---------- */
        double xError = pose.getPosition().x;
        if (Math.abs(xError) > STRAFE_TOLERANCE) {
            strafePower = Range.clip(
                    xError * kP_strafe,
                    -MAX_STRAFE_POWER,
                    MAX_STRAFE_POWER
            );
        }

        /* ---------- TURRET ---------- */
        double yawError = -pose.getOrientation().getYaw(); //Turret yaw is opposite to camera yaw
        if (Math.abs(yawError) > TURRET_TOLERANCE) {
            turretPower = Range.clip(
                    yawError * kP_turret,
                    -MAX_TURRET_POWER,
                    MAX_TURRET_POWER
            );
        }

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
