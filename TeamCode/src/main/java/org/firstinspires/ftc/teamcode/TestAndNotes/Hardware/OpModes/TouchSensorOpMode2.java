package org.firstinspires.ftc.teamcode.TestAndNotes.Hardware.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TestAndNotes.Hardware.Mechanisms.ProgrammingBoard2;

@TeleOp
public class TouchSensorOpMode2 extends OpMode {

    ProgrammingBoard2 board = new ProgrammingBoard2();

    @Override
    public void init(){
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("Touch pressed", board.isTouchSensorPressed());
        telemetry.addData("Touch released", board.isTouchSensorReleased());

        telemetry.addData("Touch (Not) Pressed", board.isTouchSensorPressedText());
    }
}
