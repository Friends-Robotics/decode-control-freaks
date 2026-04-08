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
    public final DcMotorEx frontLeftMotor;
    public final DcMotorEx backLeftMotor;
    public final DcMotorEx frontRightMotor;
    public final DcMotorEx backRightMotor;

    // Intake
    public final DcMotorEx intakeMotor;
    public final Servo feeder;

    // Shooter
    public final DcMotorEx shooterMotor1;
    public final DcMotorEx shooterMotor2;
    public final DcMotorEx turretMotor;
    public final Servo hood;

    public final GoBildaPinpointDriver pinpoint;

    public final Limelight3A limelight;

    public RobotHardware(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        // RUN_WITHOUT_ENCODER tells the builtin motor PID not to run
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FRM");
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FLM");
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BRM");
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BLM");
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        feeder = hardwareMap.get(Servo.class, "Feeder");

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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