package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;

@Config
@TeleOp(name = "Shooter Motor Test", group = "Calibration")
public class ShooterCalibration extends LinearOpMode {
    private RobotHardware robotHardware;
    private Robot robot;

    private final PIDFController shooterPID = new PIDFController(
            RobotConstants.Shooter.kP,
            RobotConstants.Shooter.kI,
            RobotConstants.Shooter.kD,
            RobotConstants.Shooter.kS,
            RobotConstants.Shooter.kV,
            RobotConstants.Shooter.iLimit
    );

    public static double targetRPM = 0;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);

        telemetry.addLine("Ready for Shooter Calibration");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            shooterPID.setPIDF(
                    RobotConstants.Shooter.kP,
                    RobotConstants.Shooter.kI,
                    RobotConstants.Shooter.kD,
                    RobotConstants.Shooter.kS,
                    RobotConstants.Shooter.kV
            );

            double targetVelocity = (targetRPM * RobotConstants.Shooter.SHOOTER_TICKS_PER_REV) / 60.0;

            double currentVelocity = robot.getShooterVelocity();

            double shooterPower = shooterPID.calculate(targetVelocity, currentVelocity);

            // 5. Apply Power & Safety
            shooterPower = Range.clip(shooterPower, 0, RobotConstants.Shooter.MAX_POWER);

            if (targetRPM <= 0) {
                robot.setShooterPower(0);
                shooterPID.reset();
            } else {
                robot.setShooterPower(shooterPower);
            }

            // 6. Telemetry for Graphing in Dashboard
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", robot.getShooterRPM());
            telemetry.addData("Target Velocity (TPS)", targetVelocity);
            telemetry.addData("Current Velocity (TPS)", currentVelocity);
            telemetry.addData("Applied Power", shooterPower);
            telemetry.update();
        }
    }
}