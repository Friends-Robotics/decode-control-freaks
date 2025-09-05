package org.firstinspires.ftc.teamcode.controlfreaks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class VariablePractice extends OpMode {

    @Override
    public void init(){
        int teamNumber = 23014;
        double motorSpeed = 0.75;
        boolean clawClosed = true;

        telemetry.addData("TeamNumber", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Claw Closed", clawClosed);
    }

    public void loop(){

    }
}
