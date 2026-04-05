package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */

public class TeamHardwareMap {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    public DcMotor intakeMotor;
    public Servo feeder;

    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    public DcMotor turretMotor;
    public Servo hood;

    public GoBildaPinpointDriver pinpoint;

    public Limelight3A limelight;

    // Constants
    public double targetShooterRPM = 3300; //For close 4100 for far
    public static final double SHOOTER_TICKS_PER_REV = 15;

    public TeamHardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        feeder = hardwareMap.get(Servo.class, "Feeder");

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, "HoodServo");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    // INTAKE
    public void startIntake() { intakeMotor.setPower(0.6);}
    public void stopIntake() { intakeMotor.setPower(0.0);}

    // FEEDER
    public void resetFeed() {feeder.setPosition(0.0);} // Lowered
    public void feedBall() {feeder.setPosition(0.37);} // High

    // SHOOTER
    public void setShooterRPM(double rpm){
        double ticksPerSecond = (rpm * SHOOTER_TICKS_PER_REV) / 60.0;
        shooterMotor1.setVelocity(ticksPerSecond);
        shooterMotor2.setVelocity(ticksPerSecond);
    }

    public void setShooterPower(double power){
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    public double getShooterRPM() {
        return (shooterMotor1.getVelocity() * 60.0) / SHOOTER_TICKS_PER_REV;
    }
    public boolean shooterAtSpeed(double tolerance) {
        return Math.abs(getShooterRPM() - targetShooterRPM) <= tolerance;
    }
    public void stopShooter() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }
    public void setTurretPower(double power) { turretMotor.setPower(power); }
}