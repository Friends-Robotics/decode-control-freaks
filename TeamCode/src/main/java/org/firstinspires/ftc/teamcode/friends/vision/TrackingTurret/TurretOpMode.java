package org.firstinspires.ftc.teamcode.friends.vision.TrackingTurret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Turret Limelight PD")
public class TurretOpMode extends OpMode {

    private Limelight3A limelight;
    private TurretMechanism turret = new TurretMechanism();

    // Used to auto-update P and D
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        turret.init(hardwareMap);

        telemetry.addLine("Initialized Turret + Limelight");
    }

    @Override
    public void start(){
        turret.resetTimer();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if(result == null || !result.isValid()){
            // No target
            turret.update(0, 0);

            telemetry.addLine("No target");
            telemetry.update();
            return;
        }

        // FTC Limelight values
        double tx = result.getTx();   // horizontal offset (deg)
        double ty = result.getTy();   // vertical offset (deg)
        double ta = result.getTa();   // target area
        double tv = 1;                // valid target

        // PD update
        turret.update(tx, tv);

        /// Update P and D automatically

        // Steps through the different step sizes for precision
        if(gamepad1.circleWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Dpad left/right adjusts P gain
        if (gamepad1.dpadLeftWasPressed()) {
            turret.setKP(turret.getKP() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpadRightWasPressed()) {
            turret.setKP(turret.getKP() + stepSizes[stepIndex]);
        }

        // Dpad up/down adjusts D gain
        if (gamepad1.dpadUpWasPressed()) {
            turret.setKD(turret.getKD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setKD(turret.getKD() - stepSizes[stepIndex]);
        }

        telemetry.addData("Tuning P", "%.5f (D-pad L/R)", turret.getKP());
        telemetry.addData("Tuning D", "%.5f (D-pad U/D)", turret.getKD());
        telemetry.addData("Step Size", "%.5f (Circle Button)", stepSizes[stepIndex]);

        ///

        // Telemetry
        /*
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("ta", ta);
        telemetry.update();
         */
    }
}
