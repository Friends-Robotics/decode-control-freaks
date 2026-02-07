package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;

@TeleOp(name = "Comp")
public class Comp extends LinearOpMode {
    private static hardwareMap hwMap;
    private double speedModifier = 0.8;
    private float intakePower = -0.8f;
    private float shooterPower = 0.0f;
    private float servoPosition = 0.36f;

    private final Gamepad currentGp1 = new Gamepad();
    private final Gamepad previousGp1 = new Gamepad();
    private final Gamepad currentGp2 = new Gamepad();
    private final Gamepad previousGp2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new hardwareMap(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            updateGamepads();

            handleDrive();
            handleIntake();
            handleShooter();
            handleUptake();

            sendTelemetry();
        }
    }
    private void updateGamepads() {
        previousGp1.copy(currentGp1);
        currentGp1.copy(gamepad1);

        previousGp2.copy(currentGp2);
        currentGp2.copy(gamepad2);
    }
    private void handleDrive() {
        if (currentGp1.touchpad && !previousGp1.touchpad) {
            speedModifier = (speedModifier == 0.8) ? 1.0 : 0.8;
        }

        double y  = -currentGp1.left_stick_y;
        double x  = currentGp1.left_stick_x * 1.1;
        double rx = currentGp1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        hwMap.frontLeftMotor.setPower(fl * speedModifier);
        hwMap.backLeftMotor.setPower(bl * speedModifier);
        hwMap.frontRightMotor.setPower(fr * speedModifier);
        hwMap.backRightMotor.setPower(br * speedModifier);
    }
    private void handleIntake() {
        if (currentGp1.right_trigger > 0.8 && previousGp1.right_trigger <= 0.8) {
            intakePower = 0.8f;
        }

        if (currentGp1.left_trigger > 0.8 && previousGp1.left_trigger <= 0.8) {
            intakePower = -0.8f;
        }

        if (currentGp1.x) {
            hwMap.intakeMotor.setPower(intakePower);
        } else {
            hwMap.intakeMotor.setPower(0);
        }
    }
    private void handleUptake(){
        if (currentGp2.dpad_up && !previousGp2.dpad_up) {
            servoPosition = 0.33f;
        }

        if (currentGp2.dpad_down && !previousGp2.dpad_down) {
            servoPosition = 0.65f;
        }

        hwMap.uptake1.setPosition(servoPosition);
        hwMap.uptake2.setPosition(servoPosition);
    }
    private void handleShooter() {
        if (currentGp2.dpad_up && !previousGp2.dpad_up) {
            shooterPower = 0.8F;
        }

        if (currentGp2.dpad_down && !previousGp2.dpad_down) {
            shooterPower = 0.6F;
        }


        if (currentGp2.touchpad) {
            hwMap.shooterMotor1.setPower(shooterPower);
            hwMap.shooterMotor2.setPower(shooterPower);
        } else {
            hwMap.shooterMotor1.setPower(0);
            hwMap.shooterMotor2.setPower(0);
        }
    }
    private float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
    private void sendTelemetry() {
        telemetry.addData("Drive Speed", speedModifier);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Servo Position", servoPosition);
        telemetry.update();
    }
}