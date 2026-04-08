package org.firstinspires.ftc.teamcode.friends.controllers;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.friends.helpers.LowPassFilter;

public class GoalFusion {
    private final LowPassFilter txFilter = new LowPassFilter(RobotConstants.Vision.LPF_GAIN);
    private double lastGoalX = 0, lastGoalY = 0;
    private boolean hasSeenTarget = false;

    public GoalEstimate update(LLResult res, Pose2D pose, double turretAngle) {
        double rX = pose.getX(DistanceUnit.INCH);
        double rY = pose.getY(DistanceUnit.INCH);

        if (res != null && res.isValid()) {
            hasSeenTarget = true;

            // 1. Calculate Distance from Vertical Offset
            double dist = (RobotConstants.Vision.TARGET_HEIGHT - RobotConstants.Vision.CAMERA_HEIGHT) /
                    Math.tan(Math.toRadians(RobotConstants.Vision.CAMERA_ANGLE + res.getTy()));

            // 2. Filter the Vision Error
            double filteredTx = txFilter.estimate(res.getTx());

            // 3. Update Field Map Coordinates
            double absoluteAngle = Math.toRadians(turretAngle - filteredTx);
            lastGoalX = rX + Math.cos(absoluteAngle) * dist;
            lastGoalY = rY + Math.sin(absoluteAngle) * dist;

            return new GoalEstimate(-filteredTx, dist, true);

        } else if (hasSeenTarget) {
            // 4. INTERPOLATION (Vision Lost)
            double dx = lastGoalX - rX;
            double dy = lastGoalY - rY;
            double dist = Math.hypot(dx, dy);

            double targetAngle = Math.toDegrees(Math.atan2(dy, dx));
            double error = normalizeDegrees(targetAngle - turretAngle);

            return new GoalEstimate(error, dist, true);
        }

        return GoalEstimate.empty();
    }

    private double normalizeDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }
}