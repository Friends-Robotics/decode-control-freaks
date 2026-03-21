package org.firstinspires.ftc.teamcode.friends.tests;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.comp.Helpers;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterController {
    OdometryShooter odometryShooter;

    enum State {
        IDLE,
        SPINNING_UP,
        RAISING_RAMP,
        FEEDING,
        SPACING,
        RECOVERING
    }

    State currentState = State.IDLE;
    ElapsedTime timer = new ElapsedTime();
    int ballsToShoot = 0;
    int ballsShot = 0;

    // Tunables
    double rampUpTime = 1.0;
    double feedTime = 0.25;
    double spacingTime = 0.10;
    double minRecoverTime = 0.15;
    double intakePower = 0.8;
    double reversePower = -0.25;
    public double hoodPos = 0;
    double targetShooterRPM;


    // --- Start shooting with count + hood angle ---
    public void startShooting(int count, double hood, double targetRPM) {
        ballsToShoot = count;
        ballsShot = 0;           // reset
        hoodPos = hood;
        targetShooterRPM = targetRPM;
        currentState = State.SPINNING_UP;
        timer.reset();
    }

    // --- Main state machine ---
    public void update(hardwareMap robot, Helpers helpers, VisionAlign vision) {
        double rpmScale = robot.targetShooterRPM / 3300.0; // base RPM for close shot
        double adjustedFeedTime = feedTime / rpmScale;
        double adjustedSpacingTime = spacingTime / rpmScale;
        double adjustedReversePower = reversePower / rpmScale;

        switch (currentState) {

            case IDLE:
                robot.intakeMotor.setPower(0);
                robot.stopShooter();
                robot.resetFeed();
                break;

            case SPINNING_UP:
                robot.setShooterRPM(targetShooterRPM);
                robot.hood.setPosition(hoodPos);

                if (robot.shooterAtSpeed(50) && timer.seconds() > 0.2) {
                    currentState = State.RAISING_RAMP;
                    timer.reset();
                }
                break;

            case RAISING_RAMP:
                // Only feed if aligned
                if (vision.isAligned) {
                    robot.feedBall();
                    if (timer.seconds() > rampUpTime) {
                        currentState = State.FEEDING;
                        timer.reset();
                    }
                }
                break;

            case FEEDING:
                robot.intakeMotor.setPower(intakePower);

                if (timer.seconds() > adjustedFeedTime) {
                    robot.intakeMotor.setPower(0);
                    robot.resetFeed();
                    ballsShot++;
                    currentState = State.SPACING;
                    timer.reset();
                }
                break;

            case SPACING:
                if (ballsShot == 1) {
                    robot.intakeMotor.setPower(adjustedReversePower);
                } else {
                    robot.intakeMotor.setPower(0);
                }

                if (timer.seconds() > adjustedSpacingTime) {
                    robot.intakeMotor.setPower(0);
                    currentState = State.RECOVERING;
                    timer.reset();
                }
                break;

            case RECOVERING:
                double movementPenalty = Math.abs(helpers.drive) + Math.abs(helpers.strafe); // moving correction
                if (robot.shooterAtSpeed(50) && timer.seconds() > minRecoverTime + movementPenalty * 0.1) {
                    ballsToShoot--;
                    if (ballsToShoot > 0) {
                        currentState = State.RAISING_RAMP;
                    } else {
                        currentState = State.IDLE;
                    }
                    timer.reset();
                }
                break;
        }
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }
}