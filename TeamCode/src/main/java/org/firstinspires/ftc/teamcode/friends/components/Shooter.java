package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo feeder;
    private final Servo hood;
    private final Limelight3A limelight;

    private final double FEEDER_UP_POSITION = 0.45;
    private final double FEEDER_DOWN_POSITION = 0.0;

    public static double SHOOTER_TICKS_PER_REV = 28;
    public static double MAX_POWER = 1;
    public static double MIN_POWER = 0.07;


    public Shooter(DcMotorEx motor1, DcMotorEx motor2, Servo feeder, Servo hood, Limelight3A limelight) {
        this.shooterMotor1 = motor1;
        this.shooterMotor2 = motor2;
        this.feeder = feeder;
        this.hood = hood;
        this.limelight = limelight;
    }

    public void setPower(double power) {
        if (power < MIN_POWER) power = 0;
        else if (power > MAX_POWER) power = MAX_POWER;

        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    public double getRPM() {
        return (getVelocity() * 60.0) / SHOOTER_TICKS_PER_REV;
    }

    public double getVelocity() {
        return (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0;
    }

    public void setHoodPosition(double position) { hood.setPosition(position); }

    public void feed() { feeder.setPosition(FEEDER_UP_POSITION); }

    public void stopFeed() { feeder.setPosition(FEEDER_DOWN_POSITION); }

    public double getCurrent(CurrentUnit currentUnit) {
        return shooterMotor1.getCurrent(currentUnit) + shooterMotor2.getCurrent(currentUnit);
    }

    public void startLimelight(boolean isBlue) {
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(isBlue ? 0 : 1);
    }
}