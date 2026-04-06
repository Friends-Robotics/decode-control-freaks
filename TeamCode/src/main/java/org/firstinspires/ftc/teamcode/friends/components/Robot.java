package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Robot {
    private RobotHardware robotHardware;

    public Turret turret;
    public MecanumDrive mecanumDrive;

    // Constants
    public double targetShooterRPM = 3300; // For close 4100 for far
    public static final double SHOOTER_TICKS_PER_REV = 15;

    public Robot(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;

        turret = new Turret(robotHardware.turretMotor);
        mecanumDrive = new MecanumDrive(robotHardware);
    }

    // Intake
    public void startIntake() { robotHardware.intakeMotor.setPower(0.6); }
    public void stopIntake() { robotHardware.intakeMotor.setPower(0.0); }

    // Feeder
    public void stopFeed() { robotHardware.feeder.setPosition(0.0); } // Lowered
    public void startFeed() { robotHardware.feeder.setPosition(0.37); } // High

    // Shooter
    public void setShooterPower(double power){
        robotHardware.shooterMotor1.setPower(power);
        robotHardware.shooterMotor2.setPower(power);
    }

    public double getShooterRPM() {
        return (robotHardware.shooterMotor1.getVelocity() * 60.0) / SHOOTER_TICKS_PER_REV;
    }

    public void setHoodPosition(double position) {
        robotHardware.hood.setPosition(position);
    }
}
