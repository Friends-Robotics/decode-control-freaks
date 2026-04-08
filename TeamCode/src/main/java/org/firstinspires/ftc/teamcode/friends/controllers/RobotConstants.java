package org.firstinspires.ftc.teamcode.friends.controllers;

import com.acmerobotics.dashboard.config.Config;

/**
 * Contains all constants that are editable through the dashboard and used in controllers.
 * All hardware constants should be stored within the component.
 * Finalised constants should be marked 'final'.
 */
public class RobotConstants {
    @Config
    public static class Drive {
        public static final double DEADBAND = 0.05;
        public static final double STRAFE_SPEED_MULTIPLIER = 1.1;
        public static final double SPEED_MULTIPLIER = 0.8;
        public static final double MAX_ACCEL = 0.3;
    }

    @Config
    public static class Turret {
        public static double kP_CLOSE = 0.0;
        public static double kP_FAR = 0.0;
        public static double kP = 0.1;
        public static double kS = 0.2;
        public static double kI = 0.0003;
        public static double kD = 0.002;
        public static double kV = 0.0;
        public static double iLimit = 0.5;

        public static double visionkP = 0.0;
        public static double visionkS = 0.0;
        public static double visionkI = 0.0;
        public static double visionkD = 0.0;
        public static double visionkV = 0.0;
        public static double visioniLimit = 0.5;

        public static double ALIGN_TOLERANCE = 2.5;
        public static double MAX_POWER = 0.9;

        public static double MIN_TRACKING_DISTANCE = 2.5; // The smallest distance away from the april tag to start tracking
        public static double MAX_TRACKING_DISTANCE = 2.5; // The furthest distance we can be from the april tag to track
    }

    @Config
    public static class Shooter {
        public static double kP = 0.01;
        public static double kS = 0.04;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kV = 0.0007;
        public static double iLimit = 0.5;

        public static double CLOSE_RPM = 2100;
        public static double FAR_RPM = 3150;
        public static double IDLE_RPM = 500;

        public static double RPM_TOLERANCE = 150;
        public static double RPM_LPF_GAIN = 0.3;

        public static double MAX_POWER = 0.9;
    }

    @Config
    public static class Vision {
        public static double LPF_GAIN = 0.3;
        public static double TARGET_HEIGHT = 29.5;
        public static double CAMERA_ANGLE = 29.5;
        public static double CAMERA_HEIGHT = 29.5;


    }
}
