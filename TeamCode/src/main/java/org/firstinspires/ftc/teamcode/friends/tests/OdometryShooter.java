package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

public class OdometryShooter {

    // --- Turret limits ---
    private final double MAX_TURRET_ANGLE = 85; // same as vision
    private final double MIN_TURRET_ANGLE = -85;

    // --- Proportional gain for odometry turret aiming ---
    private final double kOdoAim;

    // --- Shooting zones ---
    private final double CLOSE_RPM;
    private final double FAR_RPM;
    private final double CLOSE_DIST;
    private final double FAR_DIST;
    private final double CLOSE_HOOD;
    private final double FAR_HOOD;


    public OdometryShooter(double kOdoAim,double CLOSE_RPM, double FAR_RPM, double CLOSE_DIST, double FAR_DIST, double CLOSE_HOOD, double FAR_HOOD){

        this.kOdoAim = kOdoAim;
        this.CLOSE_RPM = CLOSE_RPM;
        this.FAR_RPM = FAR_RPM;
        this.CLOSE_DIST = CLOSE_DIST;
        this.FAR_DIST = FAR_DIST;
        this.CLOSE_HOOD = CLOSE_HOOD;
        this.FAR_HOOD = FAR_HOOD;
    }


    //Returns turret motor power, using odometry aiming + optional vision + driver inputs
    public double getTurretPower(Pose robotPose, Pose goalPose, double visionCorrection, double compRotate, double compStrafe, int turretTicks){

        // --- Calculate target angle ---
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = Math.toDegrees(robotPose.getHeading());
        double desiredTurretAngle = targetAngle - robotHeading;


        while (desiredTurretAngle > 180) desiredTurretAngle -= 360;
        while (desiredTurretAngle < -180) desiredTurretAngle += 360;

        // clamp to turret limits
        desiredTurretAngle = Range.clip(desiredTurretAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);

        // proportional odometry aiming
        double currentTurretAngle = turretTicks / VisionAlign.TurretConstants.TICKS_PER_DEGREE;
        double error = desiredTurretAngle - currentTurretAngle;
        double odoTurretPower = Range.clip(error * kOdoAim, -0.2, 0.2);

        // combine odometry, vision, and driver inputs
        return odoTurretPower + visionCorrection + compRotate * 0.25 + compStrafe * 0.15;
    }

    public static double getDriveRotatePower(Pose robotPose, Pose goalPose) {

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(robotPose.getHeading());

        double error = targetAngle - robotHeading;

        // normalize
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double kRotate = 0.01; // tune this

        return Range.clip(error * kRotate, -0.4, 0.4);
    }

    //Returns distance to goal
    public double getDistance(Pose robotPose, Pose goalPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    // Returns shooterRPM based on distance
    public double getTargetRPM(double distance) {
        distance = Range.clip(distance, CLOSE_DIST, FAR_DIST);
        double slopeRPM = (FAR_RPM - CLOSE_RPM) / (FAR_DIST - CLOSE_DIST);
        return CLOSE_RPM + slopeRPM * (distance - CLOSE_DIST);
    }

    //returns hoodPosition based on distance
    public double getHoodPosition(double distance) {
        distance = Range.clip(distance, CLOSE_DIST, FAR_DIST);
        double slopeHood = (FAR_HOOD - CLOSE_HOOD) / (FAR_DIST - CLOSE_DIST);
        return Range.clip(CLOSE_HOOD + slopeHood * (distance - CLOSE_DIST), 0, 0.27);
    }
}
