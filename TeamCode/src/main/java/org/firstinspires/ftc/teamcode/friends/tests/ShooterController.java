package org.firstinspires.ftc.teamcode.friends.tests;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterController {

    enum State {
        IDLE,
        SPINNING_UP,
        FEEDING,
        RECOVERING,
        RAISING_RAMP,
    }

    State currentState = State.IDLE;

    ElapsedTime timer = new ElapsedTime();

    int ballsToShoot = 0;

    // Tunable timings

    double rampUpTime = 1.0;
    double feedTime = 0.12;     // how long intake runs to push 1 ball
    double recoverTime = 0.30;  // time between shots
    double intakePower = 1.0;

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

                if (robot.shooterAtSpeed(50)) {
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

                    currentState = State.RECOVERING;
                    timer.reset();
                }
                break;

            case RECOVERING:

                if (timer.seconds() > recoverTime) {

                    ballsToShoot--;

                    if (ballsToShoot > 0) {
                        currentState = State.RAISING_RAMP;
                    } else {
                        currentState = State.IDLE;
                        robot.intakeMotor.setPower(0);
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
