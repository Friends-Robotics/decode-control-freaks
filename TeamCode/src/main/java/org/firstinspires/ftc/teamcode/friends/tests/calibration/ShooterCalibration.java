package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;

@Config
@TeleOp(name = "Shooter Calibration", group = "Calibration")
public class ShooterCalibration extends LinearOpMode {
    // This allows you to change the target RPM live from FTC Dashboard
    public static double targetRPM = 0;

    private ShooterController shooterController;

    @Override
    public void runOpMode() {
        RobotHardware robotHardware = new RobotHardware(hardwareMap);
        Robot robot = new Robot(robotHardware);

        // Use the actual controller used in TeleOp
        shooterController = new ShooterController();

        // Link telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            shooterController.updateConstants();

            // GATHER DATA
            double rawRPM = robot.shooter.getRPM();

            // INTAKE & FEEDER LOGIC
            // Allows you to test how the RPM drops when a game element is actually fired
            if (gamepad1.right_trigger > 0.1) {
                robot.shooter.feed();
                robot.intake.intake();
            } else {
                robot.shooter.stopFeed();
                robot.intake.stop();
            }

            // UPDATE CONTROLLER
            // This now uses the internal LowPassFilter and PIDF precisely like TeleOp
            double shooterPower = shooterController.update(targetRPM, rawRPM);
            robot.shooter.setPower(shooterPower);

            // TELEMETRY & DASHBOARD GRAPHING
            telemetry.addData("01 - Target RPM", targetRPM);
            telemetry.addData("02 - Raw RPM", rawRPM);
            telemetry.addData("03 - Filtered RPM", shooterController.getFilteredRPM());
            telemetry.addData("04 - Error (RPM)", targetRPM - rawRPM);
            telemetry.addData("05 - Applied Power", shooterPower);
            telemetry.addData("06 - Is Ready", shooterController.isReady() ? 1 : 0);
            telemetry.addData("07 - Motor Current (Amps)", robot.shooter.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
        }
    }
}