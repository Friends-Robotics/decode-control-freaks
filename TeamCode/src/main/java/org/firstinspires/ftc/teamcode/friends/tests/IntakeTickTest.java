package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Intake Tick Test" ,group = "Test" )
public class IntakeTickTest extends LinearOpMode {
    DcMotor Intake;

    @Override
    public void runOpMode(){
        Intake =  hardwareMap.get(DcMotor.class, "Intake");

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        telemetry.addLine("Move intake a full revolution using joystick or by hand");

        while (opModeIsActive()) {

            // Hold A to spin slowly
            if (gamepad1.a) {
                Intake.setPower(0.01);

            } else {
                Intake.setPower(0);
            }

            // Press B when one revolution is completed
            if (gamepad1.b) {
                int ticks = Intake.getCurrentPosition();

                telemetry.addLine("REVOLUTION RECORDED");
                telemetry.addData("Ticks per revolution", ticks);
                telemetry.update();

                sleep(5000);
            }

            telemetry.addData("Encoder", Intake.getCurrentPosition());
            telemetry.addLine("Hold A = spin");
            telemetry.addLine("Press B after 1 revolution");
            telemetry.update();
        }

    }

}
