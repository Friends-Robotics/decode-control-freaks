package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;

@TeleOp(name = "LimelightAutoAlign")
public class LimelightAutoPositioning extends LinearOpMode {

    // -------- Hardware --------

    hardwareMap robot;
    VisionAlign visionAlign;


    // -------- Vision Outputs --------


    @Override
    public void runOpMode() {

        visionAlign = new VisionAlign();

        // -------- Init --------
        robot.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        robot.limelight.setPollRateHz(100);
        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();

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

            LLResult result = robot.limelight.getLatestResult();

            int turretTicks = robot.turretMotor.getCurrentPosition();
            boolean visionEnabled = gamepad1.right_bumper;
            visionAlign.update(result, visionEnabled,turretTicks);

            if (gamepad1.right_bumper && result != null && result.isValid()) {
                drive  = visionAlign.drivePower;
                strafe = 0;
                rotate = 0;
            }

            robot.turretMotor.setPower(visionAlign.turretRotatePower);

            // -------- Mecanum Drive --------
            double fl = drive + strafe + rotate;
            double fr = drive - strafe - rotate;
            double bl = drive - strafe + rotate;
            double br = drive + strafe - rotate;

            fl = Range.clip(fl, -1, 1);
            fr = Range.clip(fr, -1, 1);
            bl = Range.clip(bl, -1, 1);
            br = Range.clip(br, -1, 1);

            robot.frontLeftMotor.setPower(fl);
            robot.frontRightMotor.setPower(fr);
            robot.backLeftMotor.setPower(bl);
            robot.backRightMotor.setPower(br);

            // -------- Telemetry --------
            telemetry.addData("Vision Active", gamepad1.right_bumper);
            telemetry.addData("Turret Angle  Power", visionAlign.turretPower);
            telemetry.addData("Drive Power", visionAlign.drivePower);
            telemetry.addData("Rotate Power", visionAlign.turretRotatePower);
            telemetry.update();
        }
    }

}


