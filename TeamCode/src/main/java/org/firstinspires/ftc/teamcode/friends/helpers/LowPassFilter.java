package org.firstinspires.ftc.teamcode.friends.helpers;

public class LowPassFilter {
    private double estimate = 0;
    private final double gain;
    private boolean initialized = false;

    public LowPassFilter(double gain) { this.gain = gain; }

    public double estimate(double measured) {
        if (!initialized) {
            estimate = measured;
            initialized = true;
        }
        estimate = (gain * measured) + ((1 - gain) * estimate);
        return estimate;
    }

    public void reset() {
        estimate = 0;
        initialized = false;
    }
}
