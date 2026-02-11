package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    Servo servo;
    double servoPosition = 0.0;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "Servo");

        waitForStart();
        if (isStopRequested()) return;

        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                servoPosition += 0.01;
            }

            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                servoPosition -= 0.01;
            }

            servo.setPosition(servoPosition);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}