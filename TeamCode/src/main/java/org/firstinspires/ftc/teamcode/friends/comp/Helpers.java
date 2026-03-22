package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;

public class Helpers {
    private final hardwareMap robot;

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

    public Helpers(hardwareMap robot) {
        this.robot = robot;
    }

    public void updateGamepads(Gamepad gp1, Gamepad gp2) {
        previousGp1.copy(currentGp1);
        currentGp1.copy(gp1);

        previousGp2.copy(currentGp2);
        currentGp2.copy(gp2);
    }

    public void readDriveInputs() {
        drive = -currentGp1.left_stick_y;
        strafe = -currentGp1.left_stick_x;
        rotate = currentGp1.right_stick_x;
    }

    public void applyDrive() {
        if (currentGp1.touchpad && !previousGp1.touchpad) {
            speedModifier = (speedModifier == 0.8) ? 1.0 : 0.8;
        }

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1.0);

        double fl = (drive + strafe + rotate) / denominator;
        double bl = (drive - strafe + rotate) / denominator;
        double fr = (drive - strafe - rotate) / denominator;
        double br = (drive + strafe - rotate) / denominator;

        fl = Math.max(-0.7, Math.min(fl, 0.7));
        bl = Math.max(-0.7, Math.min(bl, 0.7));
        fr = Math.max(-0.7, Math.min(fr, 0.7));
        br = Math.max(-0.7, Math.min(br, 0.7));

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

        shooterPower = clamp(shooterPower, -1.0f, 1.0f);

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

        servoPosition = clamp(servoPosition, 0f, 0.5f);
        robot.hood.setPosition(servoPosition);
    }

    public void handleTurret() {
        if (currentGp2.left_bumper && !previousGp2.left_bumper) turretPower -= 0.1f;
        if (currentGp2.right_bumper && !previousGp2.right_bumper) turretPower += 0.1f;

        turretPower = clamp(turretPower, -1.0f, 1.0f);
        robot.turretMotor.setPower(turretPower);
    }

    private float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
}