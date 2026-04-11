package org.firstinspires.ftc.teamcode.friends.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private final DcMotorEx intakeMotor;
    private final ElapsedTime pulseTimer = new ElapsedTime();
    private boolean wasPulsing = false;

    public static final double INTAKE_POWER = 0.75;
    public static final double OUTTAKE_POWER = -0.75;

    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void intake() {
        stopPulsing();
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void outtake() {
        stopPulsing();
        intakeMotor.setPower(OUTTAKE_POWER);
    }

    public void stop() {
        stopPulsing();
        intakeMotor.setPower(0.0);
    }

    /**
     * Pulses the intake motor. Starts the ON cycle immediately upon first call.
     */
    public void pulse(double onTime, double offTime) {
        // If this is the first loop cycle we are pulsing, reset the clock
        if (!wasPulsing) {
            pulseTimer.reset();
            wasPulsing = true;
        }

        double currentTime = pulseTimer.seconds();
        double cycleTime = onTime + offTime;

        if (currentTime % cycleTime < onTime) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0.0);
        }
    }

    // Helper to clear the pulsing state when switching to other modes
    private void stopPulsing() {
        wasPulsing = false;
    }

    public double getCurrent(CurrentUnit currentUnit) {
        return intakeMotor.getCurrent(currentUnit);
    }
}
