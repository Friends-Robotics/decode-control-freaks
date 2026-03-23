package org.firstinspires.ftc.teamcode.friends.vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;


public class TagPoseEstimator {

    // Simple distance model: distance = k / sqrt(ta)
    // You will need to tune this constant for your setup.
    private static final double DISTANCE_K = 56.0; // inches * sqrt(area), tune this

    // Offset from tag to goal in FIELD coordinates (top-down view)
    // You said: 6 inches right, 2 inches up from tag.
    private static final double TAG_TO_GOAL_X = 6.0; // inches
    private static final double TAG_TO_GOAL_Y = 2.0; // inches

    /**
     * Compute tag pose in field coordinates.
     *
     * @param robotPose robot pose in field coordinates (from Pinpoint/Pedro)
     * @param result    Limelight result (tx, ta, etc.)
     * @return tag pose in field coordinates
     */
    public static Pose computeTagPose(Pose robotPose, LLResult result) {
        if (result == null || !result.isValid()) {
            // Fallback: just return something near the goal or robot
            return new Pose(robotPose.getX(), robotPose.getY(), 0);
        }

        double tx = result.getTx(); // degrees
        double ta = result.getTa(); // area

        // Convert tx to radians
        double txRad = Math.toRadians(tx);

        // Direction from robot to tag in FIELD frame
        double tagDirection = robotPose.getHeading() + txRad;

        // Estimate distance from area (very rough, tune DISTANCE_K)
        double distance;
        if (ta > 0.0001) {
            distance = DISTANCE_K / Math.sqrt(ta);
        } else {
            distance = 24.0; // fallback distance in inches
        }

        // Compute tag position in field coordinates
        double tagX = robotPose.getX() + distance * Math.cos(tagDirection);
        double tagY = robotPose.getY() + distance * Math.sin(tagDirection);

        return new Pose(tagX, tagY, 0);
    }

    /**
     * Compute goal pose from tag pose using fixed offset.
     */
    public static Pose computeGoalPoseFromTag(Pose tagPose) {
        double goalX = tagPose.getX() + TAG_TO_GOAL_X;
        double goalY = tagPose.getY() + TAG_TO_GOAL_Y;
        return new Pose(goalX, goalY, 0);
    }
}
