package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.robotcore.hardware.Gamepad;

public class TeleOpConstants {
    public float drivingSpeed = 0.8f;
    public float intakePower = 0.8f;
    public float shooterPower = 0.0f;
    public float turretPower = 0.0f;
    public float servoPosition = 0.0f;
    public float drive, strafe, rotate;

    public final Gamepad currentGP1 = new Gamepad();
    public final Gamepad previousGP1 = new Gamepad();
    public final Gamepad currentGP2 = new Gamepad();
    public final Gamepad previousGP2 = new Gamepad();
}
