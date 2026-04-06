package org.firstinspires.ftc.teamcode.friends.components;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;

public class Robot {
    private RobotHardware robotHardware;

    public Turret turret;
    public MecanumDrive mecanumDrive;

    public Robot(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;

        turret = new Turret(robotHardware.turretMotor);
        mecanumDrive = new MecanumDrive(robotHardware);
    }

    // Intake Logic
    public void intake() { robotHardware.intakeMotor.setPower(0.6); }
    public void outtake() { robotHardware.intakeMotor.setPower(-0.6); }
    public void stopIntake() { robotHardware.intakeMotor.setPower(0.0); }

    // Feeder Logic (Servo)
    public void stopFeed() { robotHardware.feeder.setPosition(0.0); }
    public void startFeed() { robotHardware.feeder.setPosition(0.36); }

    // Shooter Logic
    public void setShooterPower(double power){
        robotHardware.shooterMotor1.setPower(power);
        robotHardware.shooterMotor2.setPower(power);
    }

    public double getShooterVelocity() {
        return (robotHardware.shooterMotor1.getVelocity() + robotHardware.shooterMotor2.getVelocity()) / 2.0;
    }

    public double getShooterCurrentDraw() {
        return robotHardware.shooterMotor1.getCurrent(CurrentUnit.AMPS) + robotHardware.shooterMotor2.getCurrent(CurrentUnit.AMPS);
    }

    public double getShooterRPM() {
        return (getShooterVelocity() * 60.0) / RobotConstants.Shooter.SHOOTER_TICKS_PER_REV;
    }

    // Hood Logic
    public void setHoodPosition(double position) {
        robotHardware.hood.setPosition(position);
    }
}
