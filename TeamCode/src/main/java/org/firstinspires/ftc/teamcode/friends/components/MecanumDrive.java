package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;

public class MecanumDrive {
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backRightMotor;

    private double lastFL = 0;
    private double lastBL = 0;
    private double lastFR = 0;
    private double lastBR = 0;

    public MecanumDrive(DcMotorEx frontLeftMotor, DcMotorEx backLeftMotor, DcMotorEx frontRightMotor, DcMotorEx backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.backLeftMotor = backLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backRightMotor = backRightMotor;
    }

    public void move(double drive, double strafe, double rotate) {
        drive = Math.abs(drive) < RobotConstants.Drive.DEADBAND ? 0 : drive;
        strafe = Math.abs(strafe) < RobotConstants.Drive.DEADBAND ? 0 : strafe;
        rotate = Math.abs(rotate) < RobotConstants.Drive.DEADBAND ? 0 : rotate;

        double targetFL = drive + (strafe * RobotConstants.Drive.STRAFE_SPEED_MULTIPLIER) + rotate;
        double targetBL = drive - (strafe * RobotConstants.Drive.STRAFE_SPEED_MULTIPLIER) + rotate;
        double targetFR = drive - (strafe * RobotConstants.Drive.STRAFE_SPEED_MULTIPLIER) - rotate;
        double targetBR = drive + (strafe * RobotConstants.Drive.STRAFE_SPEED_MULTIPLIER) - rotate;

        double max = Math.max(Math.abs(targetFL),
                Math.max(Math.abs(targetBL),
                        Math.max(Math.abs(targetFR),
                                Math.abs(targetBR))));

        if (max > 1.0) {
            targetFL /= max; targetBL /= max; targetFR /= max; targetBR /= max;
        }

        lastFL = ramp(lastFL, targetFL * RobotConstants.Drive.SPEED_MULTIPLIER);
        lastBL = ramp(lastBL, targetBL * RobotConstants.Drive.SPEED_MULTIPLIER);
        lastFR = ramp(lastFR, targetFR * RobotConstants.Drive.SPEED_MULTIPLIER);
        lastBR = ramp(lastBR, targetBR * RobotConstants.Drive.SPEED_MULTIPLIER);

        frontLeftMotor.setPower(lastFL);
        backLeftMotor.setPower(lastBL);
        frontRightMotor.setPower(lastFR);
        backRightMotor.setPower(lastBR);
    }

    private double ramp(double current, double target) {
        double delta = target - current;
        if (Math.abs(delta) > RobotConstants.Drive.MAX_ACCEL) {
            return current + (Math.signum(delta) * RobotConstants.Drive.MAX_ACCEL);
        } else {
            return target;
        }
    }

    public double getCurrent(CurrentUnit currentUnit) {
        return frontLeftMotor.getCurrent(currentUnit) + backLeftMotor.getCurrent(currentUnit) +
                frontRightMotor.getCurrent(currentUnit) + backRightMotor.getCurrent(currentUnit);
    }
}
