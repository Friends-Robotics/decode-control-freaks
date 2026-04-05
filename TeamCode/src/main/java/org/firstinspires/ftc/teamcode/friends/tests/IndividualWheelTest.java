package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.friends.comp.TeamHardwareMap;

@TeleOp(name = "Individual Wheel Test", group = "Testing")
public class IndividualWheelTest extends LinearOpMode {

    TeamHardwareMap robot;

    @Override
    public void runOpMode() {

        // Get motors from hardware map
        robot = new TeamHardwareMap(hardwareMap);

        telemetry.addLine("Individual Wheel Test");
        telemetry.addLine("Press gamepad buttons to test each wheel");
        telemetry.addLine("FL = Y, BL = X, FR = B, BR = A");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Reset powers
            robot.frontLeftMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            // Control each wheel individually with gamepad buttons
            if (gamepad1.y) robot.frontLeftMotor.setPower(0.5);
            if (gamepad1.x) robot.backLeftMotor.setPower(0.5);
            if (gamepad1.b) robot.frontRightMotor.setPower(0.5);
            if (gamepad1.a) robot.backRightMotor.setPower(0.5);

            // Telemetry feedback
            telemetry.addData("Front Left", robot.frontLeftMotor.getPower());
            telemetry.addData("Back Left", robot.backLeftMotor.getPower());
            telemetry.addData("Front Right", robot.frontRightMotor.getPower());
            telemetry.addData("Back Right", robot.backRightMotor.getPower());
            telemetry.update();
        }
    }
}