package org.firstinspires.ftc.teamcode.TestAndNotes.Motors.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TestAndNotes.Motors.Mechanisms.ProgrammingBoard3;

@TeleOp
public class MotorOpMode extends OpMode {
    ProgrammingBoard3 board = new ProgrammingBoard3();

    @Override
    public void init(){
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        board.setMotorSpeed(0.5);
    }
}
