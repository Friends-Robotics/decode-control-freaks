package org.firstinspires.ftc.teamcode.friends;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */
public class hardwareMap {

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor intakeMotor;
    public Servo uptake1;
    public Servo uptake2;
    public Servo turretAngle;
    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;

    // Constants
    private double targetRPM = 0;
    public static final double TICKS_PER_REV = 28;

    public hardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hardwaremap) {

        // Drive motors
        frontRightMotor = hardwaremap.get(DcMotor.class, "FRM");
        frontLeftMotor  = hardwaremap.get(DcMotor.class, "FLM");
        backRightMotor  = hardwaremap.get(DcMotor.class, "BRM");
        backLeftMotor   = hardwaremap.get(DcMotor.class, "BLM");

        // Directions (mecanum standard)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake
        intakeMotor = hardwaremap.get(DcMotor.class, "Intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Uptake servos
        uptake1 = hardwaremap.get(Servo.class, "Uptake1");
        uptake2 = hardwaremap.get(Servo.class, "Uptake2");
        uptake2.setDirection(Servo.Direction.REVERSE);

        // Turret
        turretAngle = hardwaremap.get(Servo.class, "Turret Servo");

        // Shooter motors
        shooterMotor1 = hardwaremap.get(DcMotorEx.class, "Shooter1");
        shooterMotor2 = hardwaremap.get(DcMotorEx.class, "Shooter2");

        // Shooter directions (FIXED)
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // ================= INTAKE =================
    public void startIntake(double power) {
        intakeMotor.setPower(power);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    // ================= FEEDER =================
    public void feedBall() {
        uptake1.setPosition(1.0);
        uptake2.setPosition(1.0);
    }

    public void resetFeeder() {
        uptake1.setPosition(0.0);
        uptake2.setPosition(0.0);
    }

    // ================= SHOOTER =================
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
