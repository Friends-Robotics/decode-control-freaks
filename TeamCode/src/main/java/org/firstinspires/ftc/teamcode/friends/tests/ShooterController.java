package org.firstinspires.ftc.teamcode.friends.tests;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterController {

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

    // Tunables
    double rampUpTime = 1.5;
    double feedTime = 0.25;
    double spacingTime = 0.10;
    double minRecoverTime = 0.15;

    double intakePower = 0.8;
    double reversePower = -0.2;
    int ballsShot = 0;
    double hoodPos = 0; // betweeen 1 and 2


    public void startShooting(int count, double hood) {
        ballsToShoot = count;
        hoodPos = hood;
        currentState = State.SPINNING_UP;
        timer.reset();
    }

    public void update(hardwareMap robot) {
        double rpmScale = robot.targetShooterRPM / 3300; //3300 is the targetrpm for close shooting
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
                robot.setShooterRPM(robot.targetShooterRPM);

                if (robot.shooterAtSpeed(50)&& timer.seconds() > 0.2) {
                    currentState = State.RAISING_RAMP;
                    timer.reset();
                }
                break;

            case RAISING_RAMP:
                robot.feedBall();

                if (timer.seconds() > rampUpTime) {
                    currentState = State.FEEDING;
                    timer.reset();
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
                    robot.intakeMotor.setPower(0); // no reverse
                }

                if (timer.seconds() > adjustedSpacingTime) {
                    robot.intakeMotor.setPower(0);
                    currentState = State.RECOVERING;
                    timer.reset();
                }
                break;

            case RECOVERING:

                // wait until shooter actually recovers
                if (robot.shooterAtSpeed(50) && timer.seconds() > minRecoverTime) {

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