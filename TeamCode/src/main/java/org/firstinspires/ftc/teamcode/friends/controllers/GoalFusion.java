package org.firstinspires.ftc.teamcode.friends.controllers;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.friends.helpers.LowPassFilter;
public class GoalFusion {
    private final LowPassFilter txFilter = new LowPassFilter(RobotConstants.Vision.LPF_GAIN);
    private double lastGoalX = 0, lastGoalY = 0;
    private boolean hasSeenTarget = false;

    public GoalEstimate update(LLResult result, Pose2D pose, double turretAngle) {
        double rX = pose.getX(DistanceUnit.INCH);
        double rY = pose.getY(DistanceUnit.INCH);
        double rHeading = pose.getHeading(AngleUnit.DEGREES);

        if (result != null && result.isValid()) {
            hasSeenTarget = true;

            double dist = (RobotConstants.Vision.TARGET_HEIGHT - RobotConstants.Vision.CAMERA_HEIGHT) /
                    Math.tan(Math.toRadians(RobotConstants.Vision.CAMERA_ANGLE + result.getTy()));

            double filteredTx = txFilter.estimate(result.getTx());

            double absoluteAngleRad = Math.toRadians(normalizeDegrees(rHeading + turretAngle + filteredTx));

            lastGoalX = rX + Math.sin(absoluteAngleRad) * dist;
            lastGoalY = rY + Math.cos(absoluteAngleRad) * dist;

            return new GoalEstimate(-filteredTx, dist, true);

        } else if (hasSeenTarget) {
            double dx = lastGoalX - rX;
            double dy = lastGoalY - rY;
            double dist = Math.hypot(dy, dx);

            double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

            double desiredTurretAngle = normalizeDegrees(fieldAngleToGoal - rHeading);
            double error = normalizeDegrees(desiredTurretAngle - turretAngle);

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
