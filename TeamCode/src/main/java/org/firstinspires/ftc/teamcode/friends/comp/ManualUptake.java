package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;

@TeleOp(name = "Comp 2 Controller")
public class ManualUptake extends LinearOpMode {
    private hardwareMap hwMap;
    private double driveSpeed = 0.8;
    private double intakePower = -0.8;
    private double shooterPower = 0.6;
    private double servoPosition = 0.6;
    private boolean shooterEnabled = false;

    private final Gamepad currentGp1 = new Gamepad();
    private final Gamepad previousGp1 = new Gamepad();
    private final Gamepad currentGp2 = new Gamepad();
    private final Gamepad previousGp2 = new Gamepad();

    @Override
    public void runOpMode() {
        hwMap = new hardwareMap(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            updateGamepads();

            handleDrive();
            handleIntake();
            handleShooter();

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
        // Toggle drive speed
        if (currentGp1.touchpad && !previousGp1.touchpad) {
            driveSpeed = (driveSpeed == 0.8) ? 1.0 : 0.8;
        }

        double y  = -currentGp1.left_stick_y;
        double x  = currentGp1.left_stick_x * 1.1;
        double rx = currentGp1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        hwMap.frontLeftMotor.setPower((y + x + rx) / denominator * driveSpeed);
        hwMap.backLeftMotor.setPower((y - x + rx) / denominator * driveSpeed);
        hwMap.frontRightMotor.setPower((y - x - rx) / denominator * driveSpeed);
        hwMap.backRightMotor.setPower((y + x - rx) / denominator * driveSpeed);
    }

    private void handleIntake() {
        // Change intake direction
        if (currentGp1.dpad_left && !previousGp1.dpad_left) {
            intakePower = -0.8; // IN
        }

        if (currentGp1.dpad_right && !previousGp1.dpad_right) {
            intakePower = 0.8; // OUT
        }

        // Run intake while bumper held
        if (currentGp1.right_bumper) {
            hwMap.intakeMotor.setPower(intakePower);
        } else {
            hwMap.intakeMotor.setPower(0);
        }
    }

    private void handleShooter() {
        if (currentGp2.cross && !previousGp2.circle) {
            shooterEnabled = !shooterEnabled;
        }

        if (currentGp2.cross && !previousGp2.circle) {
            shooterPower = (shooterPower == 0.6) ? 0.8 : 0.6;
        }

        double power = shooterEnabled ? shooterPower : 0;

        hwMap.shooterMotor1.setPower(power);
        hwMap.shooterMotor2.setPower(power);
    }

    private void sendTelemetry() {
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.update();
    }
}
