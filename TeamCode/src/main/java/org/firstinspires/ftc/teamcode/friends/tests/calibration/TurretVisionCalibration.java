package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.controllers.TurretController;
import org.firstinspires.ftc.teamcode.friends.helpers.LowPassFilter;

@Config
@TeleOp(name = "Turret Vision Calibration", group = "Calibration")
public class TurretVisionCalibration extends LinearOpMode {
    public static boolean forceTracking = false;

    private TurretController turretController;
    private LowPassFilter txFilter;

    @Override
    public void runOpMode() {
        RobotHardware robotHardware = new RobotHardware(hardwareMap);
        Robot robot = new Robot(robotHardware);

        // Use the actual controller and filter logic
        turretController = new TurretController();
        txFilter = new LowPassFilter(RobotConstants.Vision.LPF_GAIN);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.shooter.startLimelight(false);
        robot.turret.resetEncoder();

        telemetry.addLine("Ready for Turret Calibration");
        telemetry.addLine("Hold A: Vision Tracking");
        telemetry.addLine("Press B: Home (0 degrees)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. PUSH DASHBOARD UPDATES
            // Add updateConstants() to TurretController if you haven't yet!
            turretController.updateConstants();

            // 2. GATHER DATA
            LLResult result = robotHardware.limelight.getLatestResult();
            double turretAngle = robot.turret.getAngle();
            double turretPower;
            double rawTx = 0;
            double filteredTx = 0;

            // 3. LOGIC BRANCHING
            if (gamepad1.triangle || forceTracking) {
                // VISION TRACKING MODE
                if (result != null && result.isValid()) {
                    rawTx = result.getTx();
                    filteredTx = txFilter.estimate(rawTx);

                    turretPower = turretController.update(-filteredTx);
                } else {
                    turretPower = 0.0;
                }
            }
            else if (gamepad1.cross) {
                turretPower = turretController.update(turretAngle);
            }
            else {
                turretPower = 0.0;
                txFilter.reset();
            }

            robot.turret.setPower(turretPower);

            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Raw tx", rawTx);
            telemetry.addData("Filtered tx", filteredTx);
            telemetry.addData("Calculated Power", turretPower);
            telemetry.addData("Is Aligned", turretController.isAligned());
            telemetry.update();
        }
    }
}