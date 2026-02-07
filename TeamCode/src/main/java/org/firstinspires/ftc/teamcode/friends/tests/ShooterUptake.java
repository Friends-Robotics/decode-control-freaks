package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ShooterUptake")
public class ShooterUptake extends LinearOpMode {
    Servo servo1;
    Servo servo2;

    double servoPosition = 0.65;
    private static float shooterPower = 0.0f;

    @Override
    public void runOpMode() {
        DcMotor motor1 = hardwareMap.dcMotor.get("Motor1");
        DcMotor motor2 = hardwareMap.dcMotor.get("Motor2");

        servo1 = hardwareMap.get(Servo.class, "Servo1");
        servo2 = hardwareMap.get(Servo.class, "Servo2");

        servo2.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                servoPosition = 0.33;
            }

            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                servoPosition = 0.65;
            }

            servo1.setPosition(servoPosition);
            servo2.setPosition(servoPosition);


            if(gamepad1.touchpad) {
                motor1.setPower(shooterPower);
                motor2.setPower(shooterPower);
            } else {
                motor1.setPower(0.0f);
                motor2.setPower(0.0f);
            }

            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                shooterPower += 0.1f;
            }
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left){
                shooterPower -= 0.1f;
            }

            shooterPower = Math.min(1.0f, shooterPower);
            shooterPower = Math.max(-1.0f, shooterPower);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.update();
        }
    }
}