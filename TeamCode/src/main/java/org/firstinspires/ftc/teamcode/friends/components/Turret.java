package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Turret {
    private final DcMotorEx turretMotor;

    // 28 ticks per motor revolution, 5.23 cartridge on motor, 200 teeth on large gera, 36 teeth on the small gear
    public static final double TICKS_PER_REVOLUTION = 28 * 5.23 * (200.0 / 36.0);
    public static final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    public static final double MIN_ANGLE = -45;
    public static final double MAX_ANGLE = 45;

    public Turret(DcMotorEx turretMotor) {
        this.turretMotor = turretMotor;
    }

    public void setPower(double power) {
        double currentAngle = getAngle();

        if (currentAngle >= MAX_ANGLE && power > 0
                || currentAngle <= MIN_ANGLE && power < 0) {
            power = 0;
        }

        turretMotor.setPower(Range.clip(power, -1, 1));
    }

    public double getAngle() { return getTicks() / TICKS_PER_DEGREE; }

    public double getTicks() { return turretMotor.getCurrentPosition(); }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getCurrent(CurrentUnit currentUnit) { return turretMotor.getCurrent(currentUnit); }
}