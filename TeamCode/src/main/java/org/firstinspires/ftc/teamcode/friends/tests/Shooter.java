package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;

@TeleOp(name = "Shooter Test")
public class Shooter extends LinearOpMode {
    hardwareMap robot;
    private static float shooterPower = 0.0f;
    private static float turretPower = 0.0f;
    private static float servoPosition = 0.0f;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new hardwareMap(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            /// Shooting
            if(gamepad1.touchpad) {
                robot.setShooterRPM(robot.targetRPM);
            } else {
                robot.stopShooter();
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                robot.targetRPM += 100f;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                robot.targetRPM -= 100f;
            }

            robot.targetRPM = Range.clip(robot.targetRPM, 0, 6000);


            /// Shooter Angle

            if(currentGamepad1.left_trigger == 1.0 && currentGamepad1.right_trigger == 1.0)
                robot.feeder.setPosition(servoPosition);

            if(currentGamepad1.triangle && !(previousGamepad1.triangle))
                servoPosition += 0.05f;
            else if(currentGamepad1.cross && !(previousGamepad1.cross))
                servoPosition -= 0.05f;

            servoPosition = Math.min(1.0f, servoPosition);
            servoPosition = Math.max(0f, servoPosition);

            /// Turret Movement

            robot.
                    turretMotor.setPower(turretPower);

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

            telemetry.addData("Target RPM", robot.targetRPM);
            telemetry.addData("Current RPM", robot.getShooterRPM());
            telemetry.addData("At Speed", robot.shooterAtSpeed(50));
            telemetry.addData("Power: ", shooterPower);
            telemetry.addData("Angle: ", servoPosition);
            telemetry.update();
        }
    }
}