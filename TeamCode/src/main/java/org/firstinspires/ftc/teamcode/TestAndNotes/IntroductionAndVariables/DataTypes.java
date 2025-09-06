package org.firstinspires.ftc.teamcode.TestAndNotes.IntroductionAndVariables;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DataTypes extends OpMode {

    @Override
    public void init(){
        String name = "Harry";
        int teamNumber = 25184;
        double motorSpeed = 0.5;
        boolean touchSensorPressed = true;

        telemetry.addData("Hello", name);
        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);
    }

    @Override
    public void loop(){

    }
}
