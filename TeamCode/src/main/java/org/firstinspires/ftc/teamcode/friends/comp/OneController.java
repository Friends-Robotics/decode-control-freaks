package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;

// Abstract into methods

@TeleOp(name = "Comp")
public class OneController extends LinearOpMode {
    private static double speedModifier = 0.8;
    private static float intakePower = -0.8f;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap hwMap = new hardwareMap(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()) {

            /// Driving

            if (gamepad1.touchpad && speedModifier == 0.8) {
                speedModifier = 1.0;
            } else if (gamepad1.touchpad && speedModifier == 1.0) {
                speedModifier = 0.8;
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            hwMap.frontLeftMotor.setPower(frontLeftPower * speedModifier);
            hwMap.backLeftMotor.setPower(backLeftPower * speedModifier);
            hwMap.frontRightMotor.setPower(frontRightPower * speedModifier);
            hwMap.backRightMotor.setPower(backRightPower * speedModifier);

            /// Intake

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.right_trigger == 1 && !(previousGamepad1.right_trigger == 1)) {
                intakePower = 0.8f;
            }

            if (currentGamepad1.left_trigger == 1 && !(previousGamepad1.left_trigger == 1)) {
                intakePower = -0.8f;
            }

            if (gamepad1.x) {
                hwMap.intakeMotor.setPower(intakePower);
            } else {
                hwMap.intakeMotor.setPower(0);
            }

            telemetry.addData("Robot Speed: ", speedModifier);
            telemetry.addData("Intake Motor Speed: ", intakePower);
            telemetry.update();
        }
    }
}