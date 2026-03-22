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
    double ROTATE_TOLERANCE = 0.05;
    double DRIVE_TOLERANCE = 0.05;

    //----ODOMETRY Assist----
    double odoAssistK = 0.1; // how much odometry helps during TRACK
    double odoSearchK = 0.4; // stronger during SEARCH

    double lastTargetAngle = 0; // for prediction

    double INITIAL_SEARCH_TIME = 1.00;
    double searchPower = 0.15;
    ElapsedTime searchTimer = new ElapsedTime();

    enum State { IDLE, TRACK, SEARCH }
    State currentState = State.IDLE;

    double LOST_DELAY = 0.25;
    ElapsedTime lostTimer = new ElapsedTime();

    public void update(LLResult results, boolean enabled, int turretEncoderTicks, double odoTargetAngle) {
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
            // store last good target direction
            lastTargetAngle = results.getTx();
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
                double tx = results.getTx(); // vision horizontal offset

                // --- Combine odometry + vision ---
                double finalTargetAngle = odoTargetAngle + tx;

                // Normalize
                while (finalTargetAngle > 180) finalTargetAngle -= 360;
                while (finalTargetAngle < -180) finalTargetAngle += 360;

                // --- Error ---
                double error = finalTargetAngle - currentTurretAngle;

                while (error > 180) error -= 360;
                while (error < -180) error += 360;

                // --- Control ---
                turretRotatePower = Range.clip(error * 0.02,
                        -MAX_ROTATE_TURRET_POWER,
                        MAX_ROTATE_TURRET_POWER);

                // --- Alignment check ---
                if (Math.abs(tx) < alignmentTolerance) {
                    isAligned = true;
                }
                // --- Distance control ----
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

                break;



            case SEARCH:

                // Aim toward goal using odometry
                error = odoTargetAngle - currentTurretAngle;

                while (error > 180) error -= 360;
                while (error < -180) error += 360;

                turretRotatePower = Range.clip(error * 0.02,
                        -MAX_ROTATE_TURRET_POWER,
                        MAX_ROTATE_TURRET_POWER);

                break;
        }

        // Soft limits
        if (currentTurretAngle >= MAX_TURRET_ANGLE && turretRotatePower > 0) turretRotatePower = 0;
        if (currentTurretAngle <= MIN_TURRET_ANGLE && turretRotatePower < 0) turretRotatePower = 0;
    }
}
