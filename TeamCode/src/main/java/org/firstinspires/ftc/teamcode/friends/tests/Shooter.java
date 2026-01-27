package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Shooter Test")
public class Shooter extends LinearOpMode {
    private static float shooterPower = 0.0f;
    private static float turretPower = 0.0f;
    private static float servoPosition = 0.0f;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor1 = hardwareMap.dcMotor.get("Motor1");
        DcMotor motor2 = hardwareMap.dcMotor.get("Motor2");
        DcMotor turretMotor = hardwareMap.dcMotor.get("turretMotor");

        Servo servo = hardwareMap.servo.get("Servo");
        servo.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            /// Shooting
            if(gamepad1.touchpad) {
                motor1.setPower(shooterPower);
                motor2.setPower(shooterPower);
            } else {
                motor1.setPower(0.0f);
                motor2.setPower(0.0f);
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                shooterPower += 0.1f;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                shooterPower -= 0.1f;
            }

            shooterPower = Math.min(1.0f, shooterPower);
            shooterPower = Math.max(-1.0f, shooterPower);

            /// Shooter Angle

            if(currentGamepad1.left_trigger == 1.0 && currentGamepad1.right_trigger == 1.0)
                servo.setPosition(servoPosition);

            if(currentGamepad1.triangle && !(previousGamepad1.triangle))
                servoPosition += 0.05f;
            else if(currentGamepad1.cross && !(previousGamepad1.cross))
                servoPosition -= 0.05f;

            servoPosition = Math.min(1.0f, servoPosition);
            servoPosition = Math.max(0f, servoPosition);

            /// Turret Movement

            turretMotor.setPower(turretPower);

            if(currentGamepad1.left_bumper && !(previousGamepad1.left_bumper))
                turretPower -= 0.1f;
            if(currentGamepad1.right_bumper && !(previousGamepad1.right_bumper))
                turretPower += 0.1f;

            turretPower = Math.min(1.0f, turretPower);
            turretPower = Math.max(0.0f, turretPower);

            telemetry.addData("Power: ", shooterPower);
            telemetry.addData("Angle: ", servoPosition);
            telemetry.update();
        }
    }
}