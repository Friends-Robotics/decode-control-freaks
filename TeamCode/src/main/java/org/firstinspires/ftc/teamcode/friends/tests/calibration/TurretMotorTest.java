package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.components.Turret;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;

public class TurretMotorTest extends LinearOpMode {
    private RobotHardware robotHardware;
    private Robot robot;

    private final PIDFController turretPID = new PIDFController(
            RobotConstants.Vision.kP,
            RobotConstants.Vision.kI,
            RobotConstants.Vision.kD,
            RobotConstants.Vision.kS,
            RobotConstants.Vision.kV,
            RobotConstants.Vision.iLimit
    );

    public static double targetAngle = 0;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);
        robot.turret.resetEncoder();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Encoder Ticks", robot.turret.getTicks());
            telemetry.addData("Angle: ", robot.turret.getAngle());
            telemetry.update();

            double turretRotatePower = turretPID.calculate(targetAngle, robot.turret.getAngle());
            robot.turret.setPower(turretRotatePower);
        }
    }
}
