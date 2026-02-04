package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter Servo Test")
public class ShooterServoTest extends LinearOpMode {
    Servo servo1;
    Servo servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "Servo1");
        servo2 = hardwareMap.get(Servo.class, "Servo2");

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            telemetry.addData("Servo 1 Position: ", servo1.getPosition());
            telemetry.addData("Servo 2 Position: ", servo2.getPosition());
            telemetry.update();
        }
    }
}