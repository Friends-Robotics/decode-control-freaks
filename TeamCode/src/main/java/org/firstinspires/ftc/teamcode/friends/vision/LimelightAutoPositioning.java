package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;






@TeleOp(name = "LimelightAutoAlign")
public class LimelightAutoPositioning extends LinearOpMode {

    // -------- Hardware --------

    hardwareMap robot;
    VisionAlign visionAlign;
    public Limelight3A limelight;

    // -------- Vision Outputs --------


    @Override
    public void runOpMode() {

        robot = new hardwareMap(hardwareMap);

        visionAlign = new VisionAlign();

        // -------- Init --------
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); //Make sure to name the Ethernet Device "limelight"
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        // -------- Loop --------
        while (opModeIsActive()) {

            // Driver inputs
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double rotate =  gamepad1.right_stick_x;

            LLResult result = limelight.getLatestResult();

            int turretTicks = robot.turretMotor.getCurrentPosition();

            boolean turretLock = gamepad1.triangle;
            visionAlign.update(result, !turretLock, turretTicks);

            if (gamepad1.right_bumper && result != null && result.isValid() && !gamepad1.left_bumper) {
                drive  = visionAlign.drivePowerClose;
                strafe = 0;
                rotate = 0;
            }
            else if (gamepad1.left_bumper && result != null && result.isValid() && !gamepad1.right_bumper) {
                drive  = visionAlign.drivePowerFar;
                strafe = 0;
                rotate = 0;
            }
            else{
                    // Manual control
                    drive  = -gamepad1.left_stick_y;
                    strafe = gamepad1.left_stick_x * 1.1;
                    rotate = gamepad1.right_stick_x;
            }



            robot.turretMotor.setPower(visionAlign.turretRotatePower);

            // -------- Mecanum Drive --------

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);

            double fl = (drive + strafe + rotate) / denominator;
            double bl = (drive - strafe + rotate) / denominator;
            double fr = (drive - strafe - rotate) / denominator;
            double br = (drive + strafe - rotate) / denominator;

            robot.frontLeftMotor.setPower(fl);
            robot.backLeftMotor.setPower(bl);
            robot.frontRightMotor.setPower(fr);
            robot.backRightMotor.setPower(br);

            // -------- Telemetry --------
            telemetry.addData("Drive Close Active", gamepad1.right_bumper);
            telemetry.addData("Drive Far Active", gamepad1.left_bumper);
            telemetry.addData("Turret Angle  Power", visionAlign.turretPower);
            telemetry.addData("Drive Power", visionAlign.drivePowerClose);
            telemetry.addData("Locked", turretLock );
            telemetry.addData("State", visionAlign.currentState);
            telemetry.addData("TurretAngle", visionAlign.currentTurretAngle);
            telemetry.addData("LastXError", visionAlign.lastXError);
            telemetry.addData("TurretPower", visionAlign.turretRotatePower);
            telemetry.addData("Ta", result.getTa());
            telemetry.update();


        }
    }
}