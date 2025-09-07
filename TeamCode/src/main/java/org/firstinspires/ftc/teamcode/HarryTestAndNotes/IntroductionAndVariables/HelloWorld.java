package org.firstinspires.ftc.teamcode.HarryTestAndNotes.IntroductionAndVariables;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HelloWorld extends OpMode {

    @Override
    public void init(){
        // Sends data to the DS which allows us to debug
        telemetry.addData("Hello", "World");
    }

    @Override
    public void loop(){

    }
}
