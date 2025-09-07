package org.firstinspires.ftc.teamcode.HarryTestAndNotes.Hardware.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HarryTestAndNotes.Hardware.Mechanisms.ProgrammingBoard1;

@TeleOp
public class TouchSensorOpMode extends OpMode {
    ProgrammingBoard1 board = new ProgrammingBoard1();

    @Override
    public void init(){
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("Touch Sensor", board.getTouchSensorState());
    }
}
