package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    Servo servo1;
    Servo servo2;

    double servoPosition = 0.0; // use double for servo positions

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "Servo1");
        servo2 = hardwareMap.get(Servo.class, "Servo2");

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

            //servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

            servo1.setPosition(servoPosition);
            servo2.setPosition(servoPosition);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}
