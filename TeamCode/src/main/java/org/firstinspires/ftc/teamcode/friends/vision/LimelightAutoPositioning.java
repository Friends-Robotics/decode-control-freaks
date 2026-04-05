package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.friends.components.Robot;

import com.qualcomm.hardware.limelightvision.Limelight3A;






@TeleOp(name = "LimelightAutoAlign")
public class LimelightAutoPositioning extends LinearOpMode {

    // -------- Hardware --------

    Robot robot;
    VisionAlign visionAlign;
    public Limelight3A limelight;

    // -------- Vision Outputs --------
    int stepIndex = 0;


    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);

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
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * 1.1;;
            double rotate = gamepad1.right_stick_x;;

            LLResult result = limelight.getLatestResult();

            int turretTicks = robot.turretMotor.getCurrentPosition();

            boolean turretLock = gamepad1.triangle;


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
            if(gamepad1.a && stepIndex < 2)
            {
                stepIndex ++;
            }
            if(gamepad1.y && stepIndex > 0 )
            {
                stepIndex --;
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


            telemetry.addLine("Modifier = 1 --> kp_rotate +-0.001, Modifier = 2 --> kp_rotate +-0.01, Modifier = 3 --> kp_rotate +-0.05,");
            telemetry.addData("Kp Modifier", stepIndex);
            telemetry.update();


        }
    }
}