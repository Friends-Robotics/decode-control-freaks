package org.firstinspires.ftc.teamcode.friends.components;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Robot {
    public final Turret turret;
    public final MecanumDrive mecanumDrive;
    public final Shooter shooter;
    public final Intake intake;

    public Robot(RobotHardware robotHardware) {
        turret = new Turret(robotHardware.turretMotor);
        mecanumDrive = new MecanumDrive(robotHardware.frontLeftMotor, robotHardware.backLeftMotor, robotHardware.frontRightMotor, robotHardware.backRightMotor);
        shooter = new Shooter(robotHardware.shooterMotor1, robotHardware.shooterMotor2, robotHardware.feeder, robotHardware.hood, robotHardware.limelight);
        intake = new Intake(robotHardware.intakeMotor);
    }

    public double getCurrent(CurrentUnit currentUnit) {
        return turret.getCurrent(currentUnit) + mecanumDrive.getCurrent(currentUnit) + shooter.getCurrent(currentUnit) + intake.getCurrent(currentUnit);
    }
}
