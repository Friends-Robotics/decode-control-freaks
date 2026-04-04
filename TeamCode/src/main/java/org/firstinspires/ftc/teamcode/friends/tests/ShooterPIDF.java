package org.firstinspires.ftc.teamcode.friends.tests;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterPIDF {

    // --- Tunables ---
    public static double kP = 0.0045;
    public static double kI = 0.0;
    public static double kD = 0.000002;
    public static double kF = 0.00001;

    private double lastError = 0;
    private double integral = 0;

    private double lastTime = System.nanoTime();

    public double calculate(double targetRPM, double currentRPM) {

        double now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        double error = targetRPM - currentRPM;

        // --- Integral ---
        integral += error * dt;

        // prevent windup
        integral = Math.max(Math.min(integral, 1000), -1000);

        // --- Derivative ---
        double derivative = (error - lastError) / dt;
        lastError = error;

        // --- PIDF ---
        double output =
                kF * targetRPM +
                        kP * error +
                        kI * integral +
                        kD * derivative;

        return output;
    }
}
