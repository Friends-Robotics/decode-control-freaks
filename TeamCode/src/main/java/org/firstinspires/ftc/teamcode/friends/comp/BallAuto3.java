package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.tests.ShooterController;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

@Autonomous(name = "3BallStrafeAuto")
public class BallAuto3 extends LinearOpMode {

    ShooterController shooterController;

    // Motors
    Robot robot;
    VisionAlign vision;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        shooterController = new ShooterController();
        vision = new VisionAlign();


        waitForStart();

        if (opModeIsActive()) {

            vision.isAligned = true;

            // Step 1: Reverse
            setDrivePower(0.5, 0.5);
            sleep(500);
            stopDrive();

            shooterController.startShooting(3);


            // Step 3: Strafe left (for mecanum)
            robot.frontLeftMotor.setPower(0.5);
            robot.backLeftMotor.setPower(-0.5);
            robot.frontRightMotor.setPower(-0.5);
            robot.backRightMotor.setPower(0.5);
            sleep(500);  // strafe for 2 seconds
            stopDrive();
        }
    }

    private void setDrivePower(double leftPower, double rightPower) {
        robot.frontLeftMotor.setPower(leftPower);
        robot.backLeftMotor.setPower(leftPower);
        robot.frontRightMotor.setPower(rightPower);
        robot.backRightMotor.setPower(rightPower);
    }

    private void stopDrive() {
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
}

