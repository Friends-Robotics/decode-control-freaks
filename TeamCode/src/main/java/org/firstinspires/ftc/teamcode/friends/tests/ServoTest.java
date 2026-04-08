package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.helpers.Utils;

@Disabled
@TeleOp(name = "Servo Test", group = "Test")
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

            servoPosition = Range.clip(servoPosition, 0, 1);

            if (currentGamepad.left_bumper) {
                servo.setPosition(servoPosition);
            }

            telemetry.addLine("======= Servo Test ======");
            telemetry.addLine("DPAD up to add 0.05");
            telemetry.addLine("DPAD up to subtract 0.05");
            telemetry.addLine("Left bumper to go to position");
            telemetry.addData("Servo Target Position: ", servoPosition);
            telemetry.update();
        }
    }
}