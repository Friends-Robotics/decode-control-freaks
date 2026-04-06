package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Tick Finder", group = "Test")
public class TurretTickFinder extends LinearOpMode {

    DcMotor turretMotor;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "Turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        int startTicks = turretMotor.getCurrentPosition();

        telemetry.addLine("Move turret 90 degrees RIGHT using joystick");
        telemetry.update();

        while (opModeIsActive() && !gamepad1.a) {
            turretMotor.setPower(gamepad1.left_stick_x);
            telemetry.addData("Ticks", turretMotor.getCurrentPosition());
            telemetry.update();
        }

        turretMotor.setPower(0);
        int rightTicks = turretMotor.getCurrentPosition() - startTicks;

        telemetry.addData("90 Right Ticks", rightTicks);
        telemetry.addLine("Press B after moving 90 degrees LEFT");
        telemetry.update();

        while (opModeIsActive() && !gamepad1.b) {
            turretMotor.setPower(gamepad1.left_stick_x);
            telemetry.addData("Ticks", turretMotor.getCurrentPosition());
            telemetry.update();
        }

        turretMotor.setPower(0);
        int leftTicks = turretMotor.getCurrentPosition() - startTicks;

        telemetry.addData("90 Left Ticks", leftTicks);
        telemetry.update();

        sleep(10000);
    }
}
