package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

public class OdometryShooter {

    // --- Turret limits ---
    private final double MAX_TURRET_ANGLE = 85;
    private final double MIN_TURRET_ANGLE = -85;

    // --- Goal offset (TAG → GOAL CENTER) ---
    // Tune these (meters)
    private final double TAG_TO_GOAL_X; // right (+) / left (-) 6
    private final double TAG_TO_GOAL_Y; // forward/up (+) 2

    public OdometryShooter(double tagToGoalX, double tagToGoalY) {
        this.TAG_TO_GOAL_X = tagToGoalX;
        this.TAG_TO_GOAL_Y = tagToGoalY;
    }

    // 🔥 MAIN METHOD: returns desired turret angle (degrees)
    public double getTargetTurretAngle(Pose robotPose, Pose tagPose) {

        // Vector robot → tag
        double dx = tagPose.getX() - robotPose.getX();
        double dy = tagPose.getY() - robotPose.getY();

        // Add offset to get goal center
        double dxGoal = dx + TAG_TO_GOAL_X;
        double dyGoal = dy + TAG_TO_GOAL_Y;

        // Angle to goal (field frame)
        double targetAngle = Math.toDegrees(Math.atan2(dyGoal, dxGoal));

        // Robot heading
        double robotHeading = Math.toDegrees(robotPose.getHeading());

        // Convert to turret-relative angle
        double desiredTurretAngle = targetAngle - robotHeading;

        // Normalize to [-180, 180]
        while (desiredTurretAngle > 180) desiredTurretAngle -= 360;
        while (desiredTurretAngle < -180) desiredTurretAngle += 360;

        // Clamp to turret limits
        return Range.clip(desiredTurretAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    }

    // --- Distance (for shooter tuning) ---
    public double getDistanceToGoal(Pose robotPose, Pose tagPose) {

        double dx = tagPose.getX() - robotPose.getX() + TAG_TO_GOAL_X;
        double dy = tagPose.getY() - robotPose.getY() + TAG_TO_GOAL_Y;

        return Math.hypot(dx, dy);
    }

    // --- Shooter tuning (same as before) ---
    public double getTargetRPM(double distance,
                               double CLOSE_RPM, double FAR_RPM,
                               double CLOSE_DIST, double FAR_DIST) {

        distance = Range.clip(distance, CLOSE_DIST, FAR_DIST);
        double slope = (FAR_RPM - CLOSE_RPM) / (FAR_DIST - CLOSE_DIST);

        return CLOSE_RPM + slope * (distance - CLOSE_DIST);
    }

    public double getHoodPosition(double distance,
                                  double CLOSE_HOOD, double FAR_HOOD,
                                  double CLOSE_DIST, double FAR_DIST) {

        distance = Range.clip(distance, CLOSE_DIST, FAR_DIST);
        double slope = (FAR_HOOD - CLOSE_HOOD) / (FAR_DIST - CLOSE_DIST);

        return Range.clip(
                CLOSE_HOOD + slope * (distance - CLOSE_DIST),
                0,
                0.27
        );
    }
}
