package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Flywheel Tick Test", group="Test")
public class FlywheelTickTest extends LinearOpMode {
    //TeamHardwareMap robot;

    @Override
    public void runOpMode() {

        //robot = new TeamHardwareMap(TeamHardwareMap);#

        DcMotorEx shooterMotor1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        DcMotorEx shooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        // Reset encoder
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Hold A to spin slowly
            if (gamepad1.a) {
                shooterMotor1.setPower(0.01);
                shooterMotor2.setPower(0.01);

            } else {
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
            }

            // Press B when one revolution is completed
            if (gamepad1.b) {
                int ticks = shooterMotor1.getCurrentPosition();

                telemetry.addLine("REVOLUTION RECORDED");
                telemetry.addData("Ticks per revolution", ticks);
                telemetry.update();

                sleep(5000);
            }

            telemetry.addData("Encoder", shooterMotor1.getCurrentPosition());
            telemetry.addLine("Hold A = spin");
            telemetry.addLine("Press B after 1 revolution");
            telemetry.update();
        }
    }
}
