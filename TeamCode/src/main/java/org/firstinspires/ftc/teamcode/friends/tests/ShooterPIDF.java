package org.firstinspires.ftc.teamcode.friends.tests;

public class ShooterPIDF {

    // --- Tunables ---
    public double kP = 0.0002;
    public double kI = 0.0;
    public double kD = 0.00005;
    public double kF = 0.00025;

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
