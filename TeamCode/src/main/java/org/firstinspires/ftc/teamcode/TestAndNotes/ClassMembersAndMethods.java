package org.firstinspires.ftc.teamcode.TestAndNotes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ClassMembersAndMethods extends OpMode {

    boolean initDone;

    @Override
    public void init(){
        telemetry.addData("init Done", initDone);
        initDone = true;
    }

    @Override
    public void loop(){
        telemetry.addData("init Done", initDone);

        double leftAmount = gamepad1.left_stick_x;
        double fwdAmount = -gamepad1.left_stick_y;

        telemetry.addData("Before x", leftAmount);
        telemetry.addData("Before y", fwdAmount);

        leftAmount = squareInputWithSign(leftAmount);
        fwdAmount = squareInputWithSign(fwdAmount);

        telemetry.addData("After x", leftAmount);
        telemetry.addData("After y", fwdAmount);
    }

    double squareInputWithSign(double input){
        double output = input * input;
        if(input < 0){
            output = output * -1;
        }
        return output;
    }
}
