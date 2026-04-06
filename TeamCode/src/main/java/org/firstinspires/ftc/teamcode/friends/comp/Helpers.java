package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.helpers.Utils;

public class Helpers {
    private final RobotHardware robot;

    public double speedModifier = 0.8;
    private float intakePower = -0.8f;
    private float shooterPower = 0.0f;
    private float turretPower = 0.0f;
    private float servoPosition = 0.0f;

    public final Gamepad currentGp1 = new Gamepad();
    public final Gamepad previousGp1 = new Gamepad();
    public final Gamepad currentGp2 = new Gamepad();
    public final Gamepad previousGp2 = new Gamepad();

    public double drive;
    public double strafe;
    public double rotate;

    public Helpers(RobotHardware robot) {
        this.robot = robot;
    }

    public void updateGamepads(Gamepad gp1, Gamepad gp2) {
        previousGp1.copy(currentGp1);
        currentGp1.copy(gp1);

        previousGp2.copy(currentGp2);
        currentGp2.copy(gp2);
    }

    public void readDriveInputs() {
        drive  = currentGp1.left_stick_y;
        strafe = -currentGp1.left_stick_x;
        rotate = -currentGp1.right_stick_x;
    }

    public void applyDrive() {
        // Base drive + rotate
        double fl = drive + strafe + rotate;
        double bl = drive - strafe + rotate;
        double fr = drive - strafe - rotate;
        double br = drive + strafe - rotate;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        robot.frontLeftMotor.setPower(fl);
        robot.backLeftMotor.setPower(bl);
        robot.frontRightMotor.setPower(fr);
        robot.backRightMotor.setPower(br);
    }

    public void handleIntake() {
        if (currentGp2.right_trigger > 0.8 && previousGp2.right_trigger <= 0.8) {
            intakePower = 0.8f;
            robot.intakeMotor.setPower(intakePower);
        }

        if (currentGp2.left_trigger > 0.8 && previousGp2.left_trigger <= 0.8) {
            intakePower = -0.8f;
            robot.intakeMotor.setPower(intakePower);

        }
    }

    public void handleShooter() {
        if (currentGp2.dpad_up && !previousGp2.dpad_up) shooterPower += 0.1f;
        if (currentGp2.dpad_down && !previousGp2.dpad_down) shooterPower -= 0.1f;

        shooterPower = Utils.clamp(shooterPower, -1.0f, 1.0f);

        if (currentGp2.touchpad) {
            robot.shooterMotor1.setPower(shooterPower);
            robot.shooterMotor2.setPower(shooterPower);
        } else {
            robot.shooterMotor1.setPower(0);
            robot.shooterMotor2.setPower(0);
        }
    }

    public void handleShooterAngle() {
        if (currentGp2.triangle && !previousGp2.triangle) servoPosition += 0.05f;
        if (currentGp2.cross && !previousGp2.cross) servoPosition -= 0.05f;

        servoPosition = Utils.clamp(servoPosition, 0f, 0.5f);
        robot.hood.setPosition(servoPosition);
    }

    public void handleTurret() {
        if (currentGp2.left_bumper && !previousGp2.left_bumper) turretPower -= 0.1f;
        if (currentGp2.right_bumper && !previousGp2.right_bumper) turretPower += 0.1f;

        turretPower = Utils.clamp(turretPower, -1.0f, 1.0f);
        robot.turretMotor.setPower(turretPower);
    }
}