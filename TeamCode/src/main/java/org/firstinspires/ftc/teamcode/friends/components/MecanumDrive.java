package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MecanumDrive {
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backRightMotor;

    public static final double DEADBAND = 0.05;
    public static final double STRAFE_SPEED_MULTIPLIER = 1.1;
    public static final double SPEED_MULTIPLIER = 0.9;
    public static final double MAX_ACCEL = 0.3;

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
        drive = Math.abs(drive) < MecanumDrive.DEADBAND ? 0 : drive;
        strafe = Math.abs(strafe) < MecanumDrive.DEADBAND ? 0 : strafe;
        rotate = Math.abs(rotate) < MecanumDrive.DEADBAND ? 0 : rotate;

        double targetFL = drive + (strafe * MecanumDrive.STRAFE_SPEED_MULTIPLIER) + rotate;
        double targetBL = drive - (strafe * MecanumDrive.STRAFE_SPEED_MULTIPLIER) + rotate;
        double targetFR = drive - (strafe * MecanumDrive.STRAFE_SPEED_MULTIPLIER) - rotate;
        double targetBR = drive + (strafe * MecanumDrive.STRAFE_SPEED_MULTIPLIER) - rotate;

        double max = Math.max(Math.abs(targetFL),
                Math.max(Math.abs(targetBL),
                        Math.max(Math.abs(targetFR),
                                Math.abs(targetBR))));

        if (max > 1.0) {
            targetFL /= max; targetBL /= max; targetFR /= max; targetBR /= max;
        }

        lastFL = ramp(lastFL, targetFL * MecanumDrive.SPEED_MULTIPLIER);
        lastBL = ramp(lastBL, targetBL * MecanumDrive.SPEED_MULTIPLIER);
        lastFR = ramp(lastFR, targetFR * MecanumDrive.SPEED_MULTIPLIER);
        lastBR = ramp(lastBR, targetBR * MecanumDrive.SPEED_MULTIPLIER);

        frontLeftMotor.setPower(lastFL);
        backLeftMotor.setPower(lastBL);
        frontRightMotor.setPower(lastFR);
        backRightMotor.setPower(lastBR);
    }

    private double ramp(double current, double target) {
        double delta = target - current;
        if (Math.abs(delta) > MecanumDrive.MAX_ACCEL) {
            return current + (Math.signum(delta) * MecanumDrive.MAX_ACCEL);
        } else {
            return target;
        }
    }

    public double getCurrent(CurrentUnit currentUnit) {
        return frontLeftMotor.getCurrent(currentUnit) + backLeftMotor.getCurrent(currentUnit) +
                frontRightMotor.getCurrent(currentUnit) + backRightMotor.getCurrent(currentUnit);
    }
}
