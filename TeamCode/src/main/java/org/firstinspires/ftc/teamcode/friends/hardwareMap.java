package org.firstinspires.ftc.teamcode.friends;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */

public class hardwareMap {

    /*
        -----------------------------------------------------------------------
        | FRM               | Front Right Wheel     | Control Hub Motor 0     |
        --------------------+-----------------------+--------------------------
        | BRM               | Back Right Wheel      | Control Hub Motor 1     |
        --------------------+-----------------------+--------------------------
        | BLM               | Back Left Wheel       | Control Hub Motor 2     |
        --------------------+-----------------------+--------------------------
        | FLM               | Front Left Wheel      | Control Hub Motor 3     |
        -----------------------------------------------------------------------
        | Intake            | Intake Motor          | Expansion Hub Motor 1   |
        -----------------------------------------------------------------------
     */

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor intakeMotor;
    public Servo feeder;
    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;

    //Constants
    private double targetRPM = 0;
    public static final double TICKS_PER_REV = 28;



    public hardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hardwaremap) {

        frontRightMotor = hardwaremap.get(DcMotor.class, "FRM");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor = hardwaremap.get(DcMotor.class, "FLM");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor = hardwaremap.get(DcMotor.class, "BRM");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor = hardwaremap.get(DcMotor.class, "BLM");
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor = hardwaremap.get(DcMotor.class, "Intake");
        feeder = hardwaremap.get(Servo.class, "Feeder");

        shooterMotor1 = hardwaremap.get(DcMotorEx.class, "Shooter1");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor2 = hardwaremap.get(DcMotorEx.class, "Shooter2");
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    //INTAKE
    public void startIntake() { intakeMotor.setPower(1.0);}
    public void stopIntake() { intakeMotor.setPower(0.0);}

    //FEEDER
    public void feedBall() {feeder.setPosition(1.0);}
    public void resetFeeder() {feeder.setPosition(0.0);}

    //SHOOTER
    public void setShooterRPM(double rpm){
        targetRPM = rpm;
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
        shooterMotor1.setVelocity(ticksPerSecond);
        shooterMotor2.setVelocity(ticksPerSecond);
    }
    public double getShooterRPM() {
        return (shooterMotor1.getVelocity() * 60.0) / TICKS_PER_REV;
    }
    public boolean shooterAtSpeed(double tolerance) {
        return Math.abs(getShooterRPM() - targetRPM) <= tolerance;
    }
    public void stopShooter() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }
}