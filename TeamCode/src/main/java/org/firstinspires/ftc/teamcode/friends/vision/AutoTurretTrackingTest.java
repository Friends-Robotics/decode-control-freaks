package org.firstinspires.ftc.teamcode.friends.vision;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;

@TeleOp(name = "AutoTurretTest")
public class AutoTurretTrackingTest extends LinearOpMode{

    hardwareMap robot;
    VisionAlign visionAlign;


    @Override
    public void runOpMode() {

        visionAlign = new VisionAlign();

        // -------- Init --------
        robot.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        robot.limelight.setPollRateHz(100);
        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            LLResult result = robot.limelight.getLatestResult();
            int turretTicks = robot.turretMotor.getCurrentPosition();
            boolean visionEnabled = gamepad1.right_bumper;
            visionAlign.update(result, visionEnabled, turretTicks);

            if (gamepad1.right_bumper && result != null && result.isValid()) {
                robot.turretMotor.setPower(visionAlign.turretRotatePower);
            }

            //TELEMETRY
            telemetry.addData("Vision Active", gamepad1.right_bumper);
            telemetry.addData("Rotate Power", visionAlign.turretRotatePower);


        }


    }
}
