package org.firstinspires.ftc.teamcode.friends.vision.TrackingTurret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Turret Limelight PD")
public class TurretOpMode extends OpMode {

    private Limelight3A limelight;
    private TurretMechanism turret = new TurretMechanism();

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

        // Telemetry
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("ta", ta);
        telemetry.update();
    }
}
