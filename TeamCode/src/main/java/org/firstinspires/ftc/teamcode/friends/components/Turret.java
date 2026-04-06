package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    private final DcMotorEx motor;

    private static final double RIGHT_TICKS = 193.0;
    private static final double LEFT_TICKS = -193.0;
    private static final double TICKS_PER_DEGREE = (RIGHT_TICKS - LEFT_TICKS) / 180.0;

    public static final double MIN_ANGLE = -42.5;
    public static final double MAX_ANGLE = 42.5;

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