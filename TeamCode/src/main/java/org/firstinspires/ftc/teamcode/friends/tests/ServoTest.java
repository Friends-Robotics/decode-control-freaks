package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    Servo servo;
    float servoPosition = 0.0f;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "Servo");

        waitForStart();

        if(isStopRequested()) return;

        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        while(opModeIsActive()){
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if(currentGamepad.dpad_up && !(previousGamepad.dpad_up))
                servoPosition += 0.05f;
            if(currentGamepad.dpad_down && !(previousGamepad.dpad_down))
                servoPosition -= 0.05f;

            servoPosition = Math.min(1.0f, servoPosition);
            servoPosition = Math.max(0.0f, servoPosition);

            telemetry.addData("Servo Position: ", servoPosition);
            telemetry.update();

            servo.setPosition(servoPosition);
        }
    }
}