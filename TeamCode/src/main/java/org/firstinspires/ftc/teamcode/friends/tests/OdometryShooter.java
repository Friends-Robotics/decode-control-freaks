package org.firstinspires.ftc.teamcode.friends.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

/**
 * OdometryShooter:
 * Computes ONLY:
 *  - angle offset between tag and goal (from robot POV)
 *  - distance from robot to goal
 *  - interpolated shooter RPM
 *  - interpolated hood position
 *
 * Tag→Goal offset is handled in TagPoseEstimator.
 * This class does NOT store any offsets.
 */
public class OdometryShooter {

    public OdometryShooter() {
        // No parameters needed anymore
    }

    /**
     * Computes the angular offset between:
     *  - robot → tag direction
     *  - robot → goal direction
     *
     * This is the small correction added to vision tx.
     */
    private double lastOffset = 0;
    public double getTagToGoalOffset(Pose robotPose, Pose tagPose, Pose goalPose) {

        // Robot → Tag vector
        double dxTag = tagPose.getX() - robotPose.getX();
        double dyTag = tagPose.getY() - robotPose.getY();
        double tagAngle = Math.toDegrees(Math.atan2(dyTag, dxTag));

        // Robot → Goal vector
        double dxGoal = goalPose.getX() - robotPose.getX();
        double dyGoal = goalPose.getY() - robotPose.getY();
        double goalAngle = Math.toDegrees(Math.atan2(dyGoal, dxGoal));

        // Offset = goalAngle - tagAngle
        double offset = goalAngle - tagAngle;

        // Normalize to [-180, 180]
        while (offset > 180) offset -= 360;
        while (offset < -180) offset += 360;

        offset = Range.clip(offset, -15, 15);

        offset = 0.8 * lastOffset + 0.2 * offset;
        lastOffset = offset;

        return offset;
    }

    /**
     * Computes straight-line distance from robot → goal.
     * Used for RPM and hood interpolation.
     */
    public double getDistanceToGoal(Pose robotPose, Pose goalPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Linearly interpolates shooter RPM based on distance.
     *
     * @param distance  current distance to goal
     * @param minRPM    RPM at minDist
     * @param maxRPM    RPM at maxDist
     * @param minDist   lower bound distance
     * @param maxDist   upper bound distance
     */
    public double getTargetRPM(double distance, double minRPM, double maxRPM,
                               double minDist, double maxDist) {

        double t = (distance - minDist) / (maxDist - minDist);
        t = Math.max(0, Math.min(1, t));  // clamp 0–1

        return minRPM + t * (maxRPM - minRPM);
    }

    /**
     * Linearly interpolates hood servo position based on distance.
     *
     * @param distance  current distance to goal
     * @param minHood   hood position at minDist
     * @param maxHood   hood position at maxDist
     * @param minDist   lower bound distance
     * @param maxDist   upper bound distance
     */
    public double getHoodPosition(double distance, double minHood, double maxHood,
                                  double minDist, double maxDist) {

        double t = (distance - minDist) / (maxDist - minDist);
        t = Math.max(0, Math.min(1, t));  // clamp 0–1

        return minHood + t * (maxHood - minHood);
    }
}