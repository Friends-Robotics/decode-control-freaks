package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;

@Config
@TeleOp(name = "Turret Vision Calibration", group = "Calibration")
public class TurretVisionCalibration extends LinearOpMode {
    public static double TARGET_OFFSET = 0;
    public static double MAX_V_POWER = 0.5;

    public static double LPF_GAIN = 0.3;

    private final PIDFController visionPID = new PIDFController(
            RobotConstants.Turret.kP,
            RobotConstants.Turret.kI,
            RobotConstants.Turret.kD,
            RobotConstants.Turret.kS,
            RobotConstants.Turret.kV,
            0.0,
            0.0,
            RobotConstants.Turret.iLimit
    );

    private double filteredTx = 0;
    private boolean lastTargetValid = false;

    @Override
    public void runOpMode() {
        RobotHardware robotHardware = new RobotHardware(hardwareMap);
        Robot robot = new Robot(robotHardware);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware.limelight.start();
        robot.turret.resetEncoder();

        waitForStart();

        while (opModeIsActive()) {
            visionPID.setPIDF(
                RobotConstants.Turret.kP,
                RobotConstants.Turret.kI,
                RobotConstants.Turret.kD,
                RobotConstants.Turret.kS,
                RobotConstants.Turret.kV,
                0.0,
                0.0
            );

            LLResult result = robotHardware.limelight.getLatestResult();
            double turretPower = 0;
            double rawTx = 0;

            if (result != null && result.isValid()) {
                rawTx = result.getTx();

                if (!lastTargetValid) {
                    filteredTx = rawTx;
                } else {
                    filteredTx = (filteredTx * (1 - LPF_GAIN)) + (rawTx * LPF_GAIN);
                }

                lastTargetValid = true;

                boolean withinTolerance = Math.abs(filteredTx) <= 2.5;

                turretPower = withinTolerance ? 0 : visionPID.calculate(TARGET_OFFSET, -filteredTx);
                turretPower = Range.clip(turretPower, -MAX_V_POWER, MAX_V_POWER);

                if (Math.abs(filteredTx - TARGET_OFFSET) < 1.0) {
                    turretPower = 0;
                }
            } else {
                lastTargetValid = false;
                visionPID.reset();
            }

            if (gamepad1.a) {
                robot.turret.setPower(turretPower);
            } else {
                robot.turret.setPower(0);
                visionPID.reset();
            }

            telemetry.addData("Status", gamepad1.a ? "TRACKING" : "IDLE (Hold A)");
            telemetry.addData("Raw tx", rawTx);
            telemetry.addData("Filtered tx", filteredTx);
            telemetry.addData("Calculated Power", turretPower);
            telemetry.update();
        }
    }
}