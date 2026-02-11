package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColourSensorTest extends OpMode {

    Colour colour = new Colour();

    @Override
    public void init(){
        colour.init(hardwareMap);
    }

    @Override
    public void loop(){
        colour.getDetectedColor(telemetry);
    }
}
