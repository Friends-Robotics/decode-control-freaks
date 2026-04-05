package org.firstinspires.ftc.teamcode.friends.helpers;

public final class Utils {
    public static double clamp(double value, double max, double min) {
        return Math.max(min, Math.min(value, max));
    }

    public static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    public static boolean withinRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    public static boolean withinTolerance(double value, double center, double tolerance) {
        return withinRange(value, center - tolerance, center + tolerance);
    }
}