package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */
public class RobotHardware {
    // Drive
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    // Intake
    public DcMotor intakeMotor;
    public Servo feeder;

    // Shooter
    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    public DcMotorEx turretMotor;
    public Servo hood;

    public GoBildaPinpointDriver pinpoint;

    public Limelight3A limelight;

    public RobotHardware(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
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
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // This tells the builtin motor PID not to run
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, "HoodServo");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
}