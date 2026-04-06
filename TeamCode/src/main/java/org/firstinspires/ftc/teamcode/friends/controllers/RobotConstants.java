package org.firstinspires.ftc.teamcode.friends.controllers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static class Drive {
        public static double SPEED_MULTIPLIER = 0.8;
    }

    public static class Vision {
        public static double kP = 0.025;
        public static double kS = 0.05;
        public static double kI = 0.005;
        public static double kD = 0.0012;
        public static double kV = 0.0;       // No velocity FF needed for turret
        public static double iLimit = 0.5;

        public static double ALIGN_TOLERANCE = 3;   // Degrees
        public static double MAX_POWER = 0.6;       // Cap for high-speed tracking
        public static double HOME_POWER = 0.40;     // Cap for gentle homing
        public static double MIN_ANGLE = -42.5;     // Software Limit Left
        public static double MAX_ANGLE = 42.5;      // Software Limit Right
    }
}
