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
    public static double SNAPSHOT_INTERVAL_MS = 150; // How often to refresh vision

    private TurretController turretController;
    private LowPassFilter txFilter;
    private double latchedError = 0;
    private long lastSnapshotTime = 0;

    @Override
    public void runOpMode() {
        RobotHardware robotHardware = new RobotHardware(hardwareMap);
        Robot robot = new Robot(robotHardware);
        turretController = new TurretController();
        txFilter = new LowPassFilter(RobotConstants.Vision.LPF_GAIN);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.shooter.startLimelight(false);
        robot.turret.resetEncoder();

        waitForStart();

        while (opModeIsActive()) {
            turretController.updateConstants();
            LLResult result = robotHardware.limelight.getLatestResult();
            double turretAngle = robot.turret.getAngle();
            double turretPower = 0;

            if (gamepad1.triangle || forceTracking) {
                if (result != null && result.isValid()) {
                    // Update the filtered error
                    double filteredTx = txFilter.estimate(result.getTx());

                    // To fight lag, we only "update" our target every 150ms
                    // while stationary. This stops the "shimmering" motor.
                    if (System.currentTimeMillis() - lastSnapshotTime > SNAPSHOT_INTERVAL_MS) {
                        latchedError = -filteredTx;
                        lastSnapshotTime = System.currentTimeMillis();
                    }

                    turretPower = turretController.update(latchedError);
                }
            } else if (gamepad1.cross) {
                // Home to 0
                turretPower = turretController.update(-turretAngle);
                latchedError = 0;
            } else {
                txFilter.reset();
                latchedError = 0;
            }

            robot.turret.setPower(turretPower);

            telemetry.addData("Mode", (gamepad1.triangle) ? "TRACKING" : "IDLE");
            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Latched Error", latchedError);
            telemetry.addData("Is Aligned", turretController.isAligned());
            telemetry.update();
        }
    }
}
