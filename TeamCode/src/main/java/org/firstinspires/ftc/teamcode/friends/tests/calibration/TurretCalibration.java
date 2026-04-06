package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.components.Turret;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;

@Config
@TeleOp(name = "Turret Motor Test", group = "Calibration")
public class TurretCalibration extends LinearOpMode {
    private RobotHardware robotHardware;
    private Robot robot;

    private final PIDFController turretPID = new PIDFController(
            RobotConstants.Turret.kP,
            RobotConstants.Turret.kI,
            RobotConstants.Turret.kD,
            RobotConstants.Turret.kS,
            RobotConstants.Turret.kV,
            RobotConstants.Turret.iLimit
    );

    public static double targetAngle = 0;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);

        telemetry.addLine("WARNING: Ensure the turret is centered");
        telemetry.update();

        waitForStart();

        // Reset the encoder
        robot.turret.resetEncoder();

        telemetry.addLine("Use the FTC dashboard to edit the target angle");
        telemetry.update();

        while (opModeIsActive()) {
            // Update the constants so the dashboard works
            turretPID.setPIDF(
                    RobotConstants.Turret.kP,
                    RobotConstants.Turret.kI,
                    RobotConstants.Turret.kD,
                    RobotConstants.Turret.kS,
                    RobotConstants.Turret.kV
            );

            targetAngle = Range.clip(targetAngle, Turret.MIN_ANGLE, Turret.MAX_ANGLE);

            double turretRotatePower = turretPID.calculate(targetAngle, robot.turret.getAngle());

            if (Math.abs(robot.turret.getAngle() - targetAngle) <= RobotConstants.Turret.ALIGN_TOLERANCE) turretRotatePower = 0;
            robot.turret.setPower(Math.min(turretRotatePower, RobotConstants.Turret.MAX_POWER));

            telemetry.addData("Target Angle: ", targetAngle);
            telemetry.addData("Angle: ", robot.turret.getAngle());
            telemetry.addData("Encoder Ticks", robot.turret.getTicks());
            telemetry.addData("Turrert rotate power", turretRotatePower);
            telemetry.update();

        }
    }
}
