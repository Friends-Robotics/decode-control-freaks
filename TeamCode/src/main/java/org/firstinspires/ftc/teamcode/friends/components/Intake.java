package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private final DcMotorEx intakeMotor;

    public static final double INTAKE_POWER = 0.75;
    public static final double STRONG_INTAKE_POWER = 0.85;
    public static final double OUTTAKE_POWER = -0.75;

    public Intake(DcMotorEx intakeMotor) { this.intakeMotor = intakeMotor; }

    public void intake() { intakeMotor.setPower(Intake.INTAKE_POWER); }

    public void outtake() { intakeMotor.setPower(Intake.OUTTAKE_POWER); }

    public void stop() { intakeMotor.setPower(0.0); }

    public double getCurrent(CurrentUnit currentUnit) {
        return intakeMotor.getCurrent(currentUnit);
    }
}
