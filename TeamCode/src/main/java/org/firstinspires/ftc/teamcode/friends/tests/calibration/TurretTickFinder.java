package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Tick Finder", group = "Calibration")
public class TurretTickFinder extends LinearOpMode {

    private DcMotor turretMotor;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotor.class, "Turret");

        telemetry.addLine("Move the turret to the center position");
        telemetry.update();

        waitForStart();

        // Reset
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Ticks: ", turretMotor.getCurrentPosition());
    }
}
