package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    private final DcMotorEx motor;

    // 28 ticks per motor revolution, 5.23 cartridge on motor, 200 teeth on large gera, 36 teeth on the small gear
    public static final double TICKS_PER_REVOLUTION = 28 * 5.23 * (200 / 36d);
    public static final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360d;

    public static final double MIN_ANGLE = -45;
    public static final double MAX_ANGLE = 45;

    public Turret(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        double currentAngle = getAngle();

        if (currentAngle >= MAX_ANGLE && power > 0
                || currentAngle <= MIN_ANGLE && power < 0) {
            power = 0;
        }

        motor.setPower(Range.clip(power, -1, 1));
    }

    public double getAngle() {
        return motor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public double getTicks() {
        return motor.getCurrentPosition();
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}