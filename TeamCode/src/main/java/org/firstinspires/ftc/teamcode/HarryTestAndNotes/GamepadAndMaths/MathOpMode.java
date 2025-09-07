package org.firstinspires.ftc.teamcode.HarryTestAndNotes.GamepadAndMaths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MathOpMode extends OpMode {

    @Override
    public void init(){}

    @Override
    public void loop(){
        double speedForwardLeft = -gamepad1.left_stick_y / 2.0;
        telemetry.addData("Left Stick y", gamepad1.left_stick_y);
        telemetry.addData("speed Forward", speedForwardLeft);

        double speedForwardRight = -gamepad1.right_stick_y / 2.0;
        telemetry.addData("Right Stick y", gamepad1.right_stick_y);
        telemetry.addData("speed Forward", speedForwardRight);

        boolean B_ButtonPressed = gamepad1.b;
        telemetry.addData("B Pressed", B_ButtonPressed);

        double difference = Math.abs(speedForwardLeft - speedForwardRight);
        telemetry.addData("Difference", difference);

        double sum = speedForwardLeft + speedForwardRight;
        telemetry.addData("Sum", sum);
    }
}
