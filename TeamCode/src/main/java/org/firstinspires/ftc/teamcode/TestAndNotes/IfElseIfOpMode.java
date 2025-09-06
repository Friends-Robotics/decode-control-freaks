package org.firstinspires.ftc.teamcode.TestAndNotes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IfElseIfOpMode extends OpMode {

    @Override
    public void init(){}

    @Override
    public void loop() {
        if (gamepad1.left_stick_x < -0.5) {
            telemetry.addData("Left stick", " is negative and large");
        } else if (gamepad1.left_stick_x < 0) {
            telemetry.addData("Left stick", " is negative and small");
        } else if (gamepad1.left_stick_x < 0.5) {
            telemetry.addData("Left stick", " is positive and small");
        } else {
            telemetry.addData("Left stick", " is positive and large");
        }
        telemetry.addData("Left stick y", gamepad1.left_stick_y);

        // If we have the amount the robot turned, but want its heading between -180 and 180
        int angle = 0;
        while(angle > 180){
            angle -= 360;
        }
        while(angle < -180){
            angle += 360;
        }

        do{
            angle++;
        }while (angle < 10);

        for(int i = 0; i < 4; i++){
            angle++;
        }

        //You may be tempted to write code like
        // while(gamepad1.a){
        //      // do something
        //}
        // That code won't work in an OpMode because gamepad1 is only updated between calls to loop()

        // Turbo Mode
        double forwardSpeed = 0;
        boolean isPressedA = gamepad1.a;
        if(!isPressedA){
            forwardSpeed = 0.5;
        }
        else if(isPressedA){
            forwardSpeed = 1;
        }
        telemetry.addData("Forward Speed", forwardSpeed);

        // Crazy Mode
        if(isPressedA){
            telemetry.addData("Left x", gamepad1.left_stick_y);
            telemetry.addData("Right x", gamepad1.left_stick_x);
        }
        else{
            telemetry.addData("Left x", gamepad1.left_stick_x);
            telemetry.addData("Left y", gamepad1.left_stick_y);
        }
    }
}
