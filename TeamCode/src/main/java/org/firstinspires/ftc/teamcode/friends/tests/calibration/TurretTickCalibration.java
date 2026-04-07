package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.friends.components.Turret;

@TeleOp(name = "Turret Tick Calibration", group = "Calibration")
public class TurretTickCalibration extends LinearOpMode {
    private DcMotor turretMotor;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotor.class, "Turret");

        telemetry.addLine("Move the turret to the center position");
        telemetry.update();

        waitForStart();

        // Reset
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive()) {
            telemetry.addData("Ticks: ", turretMotor.getCurrentPosition());
            telemetry.addData("Ticks per degree: ", Turret.TICKS_PER_DEGREE);
            telemetry.addData("Angle: ", turretMotor.getCurrentPosition() / Turret.TICKS_PER_DEGREE);
            telemetry.update();
        }
    }
}
