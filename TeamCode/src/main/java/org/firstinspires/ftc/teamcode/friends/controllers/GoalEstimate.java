package org.firstinspires.ftc.teamcode.friends.controllers;

public class GoalEstimate {
    public final double degreesFromTarget;    // Degrees off target
    public final double distance; // Inches to target
    public final boolean isEstimated;   // Can we actually see or estimate the goal?

    public GoalEstimate(double degreesFromTarget, double distance, boolean isEstimated) {
        this.degreesFromTarget = degreesFromTarget;
        this.distance = distance;
        this.isEstimated = isEstimated;
    }

    public static GoalEstimate empty() {
        return new GoalEstimate(0, 40.0, false);
    }
}
