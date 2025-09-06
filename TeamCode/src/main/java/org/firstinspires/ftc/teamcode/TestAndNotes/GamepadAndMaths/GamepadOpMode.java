package org.firstinspires.ftc.teamcode.TestAndNotes.GamepadAndMaths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamepadOpMode extends OpMode {

    @Override
    public void init(){}

    @Override
    public void loop(){
        telemetry.addData("Left Stick x", gamepad1.left_stick_x);
        telemetry.addData("Left Stick y", gamepad2.left_stick_y);
        telemetry.addData("A Button", gamepad1.a);
    }
}
