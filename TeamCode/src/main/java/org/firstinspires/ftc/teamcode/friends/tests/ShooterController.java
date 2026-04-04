package org.firstinspires.ftc.teamcode.friends.tests;
import org.firstinspires.ftc.teamcode.friends.TeamHardwareMap;
import org.firstinspires.ftc.teamcode.friends.comp.Helpers;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterController {
    private final OdometryShooter odometryShooter = new OdometryShooter();
    ShooterPIDF shooterPIDF = new ShooterPIDF();

    public boolean isBusy() {
        return false;
    }

    enum State {
        IDLE,
        SPINNING_UP,
        FEEDING
    }

    State currentState = State.SPINNING_UP;
    ElapsedTime timer = new ElapsedTime();
    int ballsToShoot = 0;
    int ballsShot = 0;

    // Tunables
    double feedTime = 4;
    double intakePower = 0.8;
    public double hoodPos;
    public double targetRPM;

    boolean shooterControlEnabled = false;

    // --- Start shooting with count + hood angle ---
    public void startShooting(int count) {
        ballsToShoot = count;
        ballsShot = 0;// reset
        shooterControlEnabled = true;
        currentState = State.SPINNING_UP;
        timer.reset();
    }

    // --- Main state machine ---
    public void update(TeamHardwareMap robot, VisionAlign vision, Helpers helpers, Pose currentPose, Pose goalPose) {


        boolean readyToShoot;

        if (vision != null) {
            readyToShoot = vision.isAligned &&
                    Math.abs(vision.turretRotatePower) < 0.05;
        } else {
            // AUTO MODE (no vision)
            readyToShoot = robot.shooterAtSpeed(75);
        }

        // --- Dynamic RPM ---
        double distance = odometryShooter.getDistanceToGoal(currentPose, goalPose);
        targetRPM = odometryShooter.getTargetRPM(distance, 3300, 4100, 60, 130);

        // --- Dynamic hood ---
        hoodPos = odometryShooter.getHoodPosition(
                distance,
                0.00, 0.25,
                60, 130
        );

        // Apply live
        double currentRPM = robot.getShooterRPM(); // or encoder method
        double power = shooterPIDF.calculate(targetRPM, currentRPM);

        if(shooterControlEnabled)
        {
            robot.setShooterPower(power);
        }
        robot.hood.setPosition(hoodPos);

        switch (currentState) {

            case IDLE:
                robot.intakeMotor.setPower(0);
                robot.stopShooter();
                robot.resetFeed();
                break;

            case SPINNING_UP:

                if (robot.shooterAtSpeed(50) && timer.seconds() > 0.2) {
                    currentState = State.FEEDING;
                    timer.reset();
                }
                break;


            case FEEDING:
                if(readyToShoot)
                {
                    robot.intakeMotor.setPower(intakePower);
                    robot.feedBall();
                    if (timer.seconds() > feedTime) {
                        robot.intakeMotor.setPower(0);
                        robot.resetFeed();
                        currentState = State.IDLE;
                        timer.reset();
                    }

                }
                break;
        }

    }
}