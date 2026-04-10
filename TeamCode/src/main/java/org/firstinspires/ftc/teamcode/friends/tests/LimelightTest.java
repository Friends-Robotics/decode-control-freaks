package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            if(result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addLine("No valid limelight data");
            }
            telemetry.update();
        }
    }
}