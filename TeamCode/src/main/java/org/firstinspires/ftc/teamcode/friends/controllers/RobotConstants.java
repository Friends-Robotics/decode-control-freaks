package org.firstinspires.ftc.teamcode.friends.controllers;

import com.acmerobotics.dashboard.config.Config;

public class RobotConstants {
    @Config
    public static class Drive {
        public static double SPEED_MULTIPLIER = 0.8;
    }

    @Config
    public static class Turret {
        public static double kP = 0.1;
        public static double kS = 0.2;
        public static double kI = 0.0003;
        public static double kD = 0.002;
        public static double kV = 0.0;       // No velocity FF needed for turret
        public static double iLimit = 0.5;

        public static double ALIGN_TOLERANCE = 2.5;   // Degrees
        public static double MAX_POWER = 0.9;       // Cap for high-speed tracking
        public static double HOME_POWER = 0.4;     // Cap for gentle homing
    }

    @Config
    public static class Shooter {
        public static double kP = 0.1;
        public static double kS = 0.2;
        public static double kI = 0.0003;
        public static double kD = 0.002;
        public static double kV = 0.0;       // No velocity FF needed for turret
        public static double iLimit = 0.5;

        public static final double SHOOTER_TICKS_PER_REV = 28; // Update this based on your motor (e.g., 28 for 5203)
        public static final double MAX_POWER = 1; // Update this based on your motor (e.g., 28 for 5203)
    }
}
