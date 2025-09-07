package org.firstinspires.ftc.teamcode.HarryTestAndNotes.Motors.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HarryTestAndNotes.Motors.Mechanisms.ProgrammingBoard4;

@TeleOp
public class MotorSensorOpMode extends OpMode {
    ProgrammingBoard4 board = new ProgrammingBoard4();

    @Override
    public void init(){
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        if(board.isTouchSensorPressed()){
            board.setMotorSpeed(0.5);
        }
        else{
            board.setMotorSpeed(0.0);
        }
        telemetry.addData("Motor Rotations", board.getMotorRotations());
    }
}
