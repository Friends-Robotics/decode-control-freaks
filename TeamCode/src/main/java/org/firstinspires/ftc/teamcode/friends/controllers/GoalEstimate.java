package org.firstinspires.ftc.teamcode.friends.controllers;

public class GoalEstimate {
    public final double degreesFromTarget;    // Degrees off target
    public final double distance; // Inches to target
    public final boolean isValid;   // Can we actually see or estimate the goal?

    public GoalEstimate(double degreesFromTarget, double distance, boolean isValid) {
        this.degreesFromTarget = degreesFromTarget;
        this.distance = distance;
        this.isValid = isValid;
    }

    public static GoalEstimate empty() {
        return new GoalEstimate(0, 40.0, false);
    }
}
