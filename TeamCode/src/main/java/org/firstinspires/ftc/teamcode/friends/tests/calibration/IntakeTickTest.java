package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Intake Tick Test" ,group = "Test" ) // Intake Ticks per REV is 150
public class IntakeTickTest extends LinearOpMode {
    DcMotor Intake;
    DcMotorEx shooterMotor1, shooterMotor2;

    @Override
    public void runOpMode(){
        Intake =  hardwareMap.get(DcMotor.class, "Intake");

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        double intakepower = 0.8;
        double MAX_INTAKE_POWER = 1.0;


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        waitForStart();
        telemetry.addLine("Move intake a full revolution using joystick or by hand");

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // Hold A to spin slowly
            if (currentGamepad1.a) {
                Intake.setPower(intakepower);

            } else {
                Intake.setPower(0);
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up && intakepower <= MAX_INTAKE_POWER) {
                intakepower += 0.05;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down && intakepower >= -MAX_INTAKE_POWER){
                intakepower -= 0.05;
            }
            if(currentGamepad1.touchpad)
            {

            }


            // Press B when one revolution is completed
            if (currentGamepad1.b) {
                int ticks = Intake.getCurrentPosition();

                telemetry.addLine("REVOLUTION RECORDED");
                telemetry.addData("Ticks per revolution", ticks);
                telemetry.update();

                sleep(5000);
            }

            telemetry.addData("Encoder", Intake.getCurrentPosition());
            telemetry.addLine("Hold A = spin");
            telemetry.addLine("Press B after 1 revolution");
            telemetry.addData("Current Intake Power", intakepower);
            telemetry.update();
        }

    }

}
