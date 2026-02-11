package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Shooter Test")
public class Shooter extends LinearOpMode {
    private static float shooterPower = 0.0f;
    //private static float turretPower = 0.0f;
    private static double servoPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor1 = hardwareMap.dcMotor.get("Motor1");
        DcMotor motor2 = hardwareMap.dcMotor.get("Motor2");

        Servo servo = hardwareMap.servo.get("Servo");
        servo.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            /// Shooter Angle

            if(currentGamepad2.dpad_up && !(previousGamepad2.dpad_up)) {
                servoPosition = 0.95;
                shooterPower = 0.7f;
            }
            else if(currentGamepad2.dpad_down && !(previousGamepad2.dpad_down)) {
                servoPosition = 0.55;
                shooterPower = 0.5f;
            }

            servo.setPosition(servoPosition);
            if (gamepad2.touchpad) {
                motor1.setPower(-shooterPower);
                motor2.setPower(-shooterPower);
            }
            else{
                motor1.setPower(0);
                motor2.setPower(0);
            }

            telemetry.addData("Shooter Power: ", shooterPower);
            telemetry.addData("Servo Angle: ", servoPosition);
            telemetry.update();
        }
    }
}