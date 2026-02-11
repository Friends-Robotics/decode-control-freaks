package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;

@TeleOp(name = "Comp")
public class Comp extends LinearOpMode {

    private hardwareMap hwMap;

    // States
    private double speedModifier = 0.8;
    private double intakePower = 0.8;
    private boolean intakeOn = false;

    private double shooterPower = 0.0;
    private double uptakeServoPosition = 0.3;
    private double turretServoPosition = 0.0;

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

            drive();
            intake();
            uptake();
            shoot();
            sendTelemetry();
        }
    }

    private void updateGamepads() {
        previousGp1.copy(currentGp1);
        currentGp1.copy(gamepad1);

        previousGp2.copy(currentGp2);
        currentGp2.copy(gamepad2);
    }
    private void drive() {
        if (currentGp1.touchpad && !previousGp1.touchpad) {
            speedModifier = (speedModifier == 0.8) ? 1.0 : 0.8;
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

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
    private void intake() {

        // Direction control (edge detection)
        if (currentGp1.right_trigger > 0.8 && previousGp1.right_trigger <= 0.8) {
            intakePower = -0.8;
            intakeOn = true;   // <<< force enable when direction pressed
        }

        if (currentGp1.left_trigger > 0.8 && previousGp1.left_trigger <= 0.8) {
            intakePower = 0.8;
            intakeOn = true;  // <<< force enable when direction pressed
        }

        // TOGGLE (enable/disable)
        if (currentGp1.cross && !previousGp1.cross) {
            intakeOn = !intakeOn;
        }

        // APPLY
        if (intakeOn) {
            hwMap.startIntake();
        } else {
            hwMap.stopIntake();
        }
    }
    private void uptake(){
        if (currentGp2.dpad_up && !previousGp2.dpad_up) {
            uptakeServoPosition = 0.65;
        }

        if (currentGp2.dpad_down && !previousGp2.dpad_down) {
            uptakeServoPosition = 0.3;
        }

        hwMap.uptake1.setPosition(uptakeServoPosition);
        hwMap.uptake2.setPosition(uptakeServoPosition);
    }
    private void shoot() {

        if (currentGp2.right_trigger >= 0.8 && !(previousGp2.right_trigger >= 0.8)) {
            turretServoPosition = 0.95;
            shooterPower = 0.7;
        }

        if (currentGp2.left_trigger >= 0.8 && !(previousGp2.left_trigger >= 0.8)) {
            turretServoPosition = 0.55;
            shooterPower = 0.5;
        }

        hwMap.turretAngle.setPosition(turretServoPosition);

        if (currentGp2.touchpad) {
            hwMap.shooterMotor1.setPower(shooterPower);
            hwMap.shooterMotor2.setPower(shooterPower);
        } else {
            hwMap.shooterMotor1.setPower(0);
            hwMap.shooterMotor2.setPower(0);
        }

        // Shooter Macro - Servo go from bottom to top then to bottom position as fast as can
    }
    private void sendTelemetry() {
        telemetry.addData("Drive Speed", speedModifier);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Uptake Servo", uptakeServoPosition);
        telemetry.addData("Turret Servo", turretServoPosition);
        telemetry.update();
    }
}
