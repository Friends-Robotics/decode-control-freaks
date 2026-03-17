package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;

@TeleOp(name = "Shooter Test")
public class Shooter extends LinearOpMode {
    hardwareMap robot;
    private static float turretPower = 0.0f;;
    private double hoodPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new hardwareMap(hardwareMap);
        ShooterController shooterController = new ShooterController();

        waitForStart();

        if (isStopRequested()) return;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // SHOOTING
            // Start shooting 3 balls
            if (gamepad1.a && !shooterController.isBusy()) {
                shooterController.startShooting(3, hoodPos);
            }

            // Update every loop
            shooterController.update(robot);
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                robot.targetShooterRPM += 100f;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                robot.targetShooterRPM -= 100f;
            }

            robot.targetShooterRPM = Range.clip(robot.targetShooterRPM, 0, 6000);


            /// Shooter Angle
            ;
            robot.hood.setPosition(hoodPos);

            if (currentGamepad1.triangle && !(previousGamepad1.triangle) && hoodPos < 1)
                hoodPos += 0.01;
            else if (currentGamepad1.square && !(previousGamepad1.square) && hoodPos > 0)
                hoodPos -= 0.01;

            /// Turret Movement

            robot.turretMotor.setPower(turretPower);

            if(currentGamepad1.left_bumper && !(previousGamepad1.left_bumper))
                turretPower -= 0.1f;
            if(currentGamepad1.right_bumper && !(previousGamepad1.right_bumper))
                turretPower += 0.1f;

            turretPower = Math.min(1.0f, turretPower);
            turretPower = Math.max(0.0f, turretPower);

            if (gamepad1.b) {
                telemetry.addLine("REVOLUTION RECORDED");
                telemetry.addData("RPM", robot.getShooterRPM());
                telemetry.update();

                sleep(5000);
            }

            telemetry.addData("Target RPM", robot.targetShooterRPM);
            telemetry.addData("Current RPM", robot.getShooterRPM());
            telemetry.addData("At Speed", robot.shooterAtSpeed(50));;
            telemetry.addData("Angle: ", hoodPos);
            telemetry.update();
        }
    }
}