package org.firstinspires.ftc.teamcode.friends.controllers;

import com.acmerobotics.dashboard.config.Config;

/**
 * Contains all constants that are editable through the dashboard and used in controllers.
 * All hardware constants should be stored within the component.
 * Finalised constants should be marked 'final'.
 */
public class RobotConstants {

    @Config
    public static class Turret {
        public static double kP = 0.0015;
        public static double kS = 0.25;
        public static double kI = 0.0;
        public static double kD = 0.0007;
        public static double kV = 0.0;
        public static double iLimit = 0.5;

        public static double ALIGN_TOLERANCE = 2;
        public static double MAX_POWER = 0.5;
    }

    @Config
    public static class Shooter {
        public static double kP = 0.00025;
        public static double kS = 0.1;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kV = 0.000285;
        public static double iLimit = 0.5;

        public static double CLOSE_RPM = 2200;
        public static double FAR_RPM = 3300;
        public static double IDLE_RPM = 500;

        public static double CLOSE_DISTANCE = 26;
        public static double FAR_DISTANCE = 109;
        public static double CLOSE_HOOD = 0.0;
        public static double FAR_HOOD = 0.2;

        public static double RPM_TOLERANCE = 100;
        public static double RPM_LPF_GAIN = 0.8;

        public static double MAX_POWER = 1.0;
    }

    @Config
    public static class Vision {
        public static double LPF_GAIN = 0.3;
        public static double TARGET_HEIGHT = 29.5;
        public static double CAMERA_ANGLE = 18.5;
        public static double CAMERA_HEIGHT = 13.75;
    }
}
