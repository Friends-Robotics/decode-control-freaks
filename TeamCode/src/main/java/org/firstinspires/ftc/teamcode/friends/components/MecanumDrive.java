package org.firstinspires.ftc.teamcode.friends.components;

public class MecanumDrive {
    private final RobotHardware robot;
    private static final double DEADBAND = 0.05;
    private static final double STRAFE_FRICTION_OFFSET = 1.1;

    public MecanumDrive(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Moves the robot
     */
    public void move(double drive, double strafe, double rotate) {
        // Apply deadband to prevent stick drift
        drive = Math.abs(drive) < DEADBAND ? 0 : drive;
        strafe = Math.abs(strafe) < DEADBAND ? 0 : strafe;
        rotate = Math.abs(rotate) < DEADBAND ? 0 : rotate;

        // The strafe friction offset compensates for speed when strafing
        double fl = drive + (strafe * STRAFE_FRICTION_OFFSET) + rotate;
        double bl = drive - (strafe * STRAFE_FRICTION_OFFSET) + rotate;
        double fr = drive - (strafe * STRAFE_FRICTION_OFFSET) - rotate;
        double br = drive + (strafe * STRAFE_FRICTION_OFFSET) - rotate;

        // Get the maximum power to normalise
        double max = Math.max(Math.abs(fl),
                     Math.max(Math.abs(bl),
                     Math.max(Math.abs(fr),
                              Math.abs(br))));

        // Normalise the powers
        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        // Set the actual motor powers
        robot.frontLeftMotor.setPower(fl);
        robot.backLeftMotor.setPower(bl);
        robot.frontRightMotor.setPower(fr);
        robot.backRightMotor.setPower(br);
    }
}