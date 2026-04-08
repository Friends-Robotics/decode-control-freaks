package org.firstinspires.ftc.teamcode.friends.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Provides helper methods
 */
public final class Utils {
    /**
     * Checks if a value lies within a range
     * @param value the value to check
     * @param min bottom of the range (inclusive)
     * @param max top of the range (inclusive)
     * @return a boolean indicating whether the value is within the range
     */
    public static boolean withinRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    /**
     * Checks if a value is within a tolerance from a target
     * @param value the value to check
     * @param target the target
     * @param tolerance the tolerance
     * @return a boolean indicating whether the value is within the tolerance of the target
     */
    public static boolean withinTolerance(double value, double target, double tolerance) {
        return withinRange(value, target - tolerance, target + tolerance);
    }

    /**
     * Standard Linear Interpolation (LERP)
     * @param start The value when t = 0
     * @param end The value when t = 1
     * @param t The percentage (0 to 1)
     * @return The interpolated value
     */
    public static double lerp(double start, double end, double t) {
        return start + (t * (end - start));
    }

    /**
     * Normalized T-Value calculation
     * @param value Current input (e.g., current distance)
     * @param min Minimum input bound
     * @param max Maximum input bound
     * @return A value clipped between 0 and 1
     */
    public static double getT(double value, double min, double max) {
        return Range.clip((value - min) / (max - min), 0, 1);
    }
}