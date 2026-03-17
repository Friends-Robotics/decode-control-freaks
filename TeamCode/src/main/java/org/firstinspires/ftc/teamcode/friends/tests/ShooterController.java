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
    double rampUpTime = 1.0;
    double feedTime = 0.25;
    double spacingTime = 0.05;
    double minRecoverTime = 0.15;

    double intakePower = 0.8;
    double reversePower = -0.2;
    int ballsShot = 0;

    public void startShooting(int count) {
        ballsToShoot = count;
        currentState = State.SPINNING_UP;
        timer.reset();
    }

    public void update(hardwareMap robot) {

        switch (currentState) {

            case IDLE:
                robot.intakeMotor.setPower(0);
                robot.resetFeed();
                break;

            case SPINNING_UP:
                robot.setShooterRPM(robot.targetShooterRPM);

                if (robot.shooterAtSpeed(40)) {
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

                if (timer.seconds() > feedTime) {
                    robot.intakeMotor.setPower(0);
                    robot.resetFeed();

                    ballsShot++;
                    currentState = State.SPACING; // 👈 go to spacing
                    timer.reset();
                }
                break;

            case SPACING:
                if (ballsShot == 1) {
                    robot.intakeMotor.setPower(reversePower);
                } else {
                    robot.intakeMotor.setPower(0); // no reverse
                }

                if (timer.seconds() > spacingTime) {
                    robot.intakeMotor.setPower(0);
                    currentState = State.RECOVERING;
                    timer.reset();
                }
                break;

            case RECOVERING:

                // wait until shooter actually recovers
                if (robot.shooterAtSpeed(40) && timer.seconds() > minRecoverTime) {

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