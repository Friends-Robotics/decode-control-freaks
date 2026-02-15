package org.firstinspires.ftc.teamcode.friends.vision.TrackingTurret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanism {

    private DcMotorEx turret;

    // PD constants
    private double kP = 0.02;     // start higher than your old value
    private double kD = 0.002;

    private double goalX = 0.0;   // target centered in camera
    private double lastError = 0.0;

    private double angleTolerance = 0.5; // degrees
    private final double MAX_POWER = 0.6;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetTimer(){
        timer.reset();
    }

    /**
     * @param tx Limelight horizontal offset (degrees)
     * @param tv target valid (1 = target detected, 0 = no target)
     */
    public void update(double tx, double tv){

        double deltaTime = timer.seconds();
        timer.reset();

        // No target → stop turret
        if(tv < 1){
            turret.setPower(0);
            lastError = 0;
            return;
        }

        // PD controller
        double error = goalX - tx;   // want tx → 0
        double pTerm = error * kP;

        double dTerm = 0;
        if(deltaTime > 0){
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        double power;
        if(Math.abs(error) < angleTolerance){
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;
    }

    // Tuning helpers
    public void setKP(double kp){ this.kP = kp; }
    public void setKD(double kd){ this.kD = kd; }
}
