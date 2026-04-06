package org.firstinspires.ftc.teamcode.friends.controllers;
import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.friends.helpers.PIDFController;

@Config
public class ShooterController {
    private final OdometryShooter odometryShooter = new OdometryShooter();

    public static double kP = 0.0002;
    public static double kI = 0.00001;
    public static double kD = 0.00005;
    public static double kS = 0.05;      // Friction of the chain
    public static double kV = 0.00025;   // Base power for RPM
    public static double iLimit = 0.2;   // Max integral power

    // Initialize with inheritance-based PIDF
    private final PIDFController shooterPIDF = new PIDFController(kP, kI, kD, kS, kV, iLimit);

    enum State { IDLE, SPINNING_UP, RAISING_RAMP, FEEDING, SPACING, RECOVERING }
    State currentState = State.IDLE;
    ElapsedTime timer = new ElapsedTime();

    int ballsToShoot = 0;
    int ballsShot = 0;

    // Timing Tunables
    double rampUpTime = 1.0;
    double feedTime = 0.25;
    double spacingTime = 0.10;
    double minRecoverTime = 0.15;
    double intakePower = 0.8;
    double reversePower = -0.25;

    public double hoodPos;
    public double targetRPM;
    boolean shooterControlEnabled = false;

    public void startShooting(int count) {
        ballsToShoot = count;
        ballsShot = 0;
        shooterControlEnabled = true;
        transitionTo(State.SPINNING_UP);
    }

    public void update(Robot robot, VisionAlign vision, Pose currentPose, Pose goalPose) {
        // double distance = odometryShooter.getDistanceToGoal(currentPose, goalPose);
        // targetRPM = 3300 + (distance * 10);
        // hoodPos = odometryShooter.getHoodPosition(distance, 0.00, 0.25, 60, 130);
        // robot.setHoodPosition(hoodPos);

        // double currentRPM = robot.getShooterRPM();
        // double power = shooterPIDF.calculate(targetRPM, currentRPM);

        // if (shooterControlEnabled && currentState != State.IDLE) {
        //     robot.setShooterPower(power);
        // } else {
        //     robot.setShooterPower(0);
        // }

        // // --- 3. DYNAMIC SCALING ---
        // double rpmScale = targetRPM / 3300.0;
        // double adjFeed = feedTime / rpmScale;
        // double adjSpacing = spacingTime / rpmScale;

        // // --- 4. STATE MACHINE ---
        // switch (currentState) {
        //     case IDLE:
        //         robot.stopIntake();
        //         robot.stopFeed();
        //         shooterControlEnabled = false;
        //         break;

        //     case SPINNING_UP:
        //         if (robot.shooterAtSpeed(50) && timer.seconds() > 0.2) {
        //             transitionTo(State.RAISING_RAMP);
        //         }
        //         break;

        //     case RAISING_RAMP:
        //         boolean ready = (vision != null) ? (vision.isAligned && Math.abs(vision.getTurretRotatePower()) < 0.05) : robot.shooterAtSpeed(75);

        //         if (ready) {
        //             robot.startFeed();
        //             if (timer.seconds() > rampUpTime) transitionTo(State.FEEDING);
        //         } else {
        //             timer.reset(); // Don't fire if we aren't locked
        //         }
        //         break;

        //     case FEEDING:
        //         robot.intakeMotor.setPower(intakePower);
        //         if (timer.seconds() > adjFeed) {
        //             robot.intakeMotor.setPower(0);
        //             robot.stopFeed();
        //             ballsShot++;
        //             transitionTo(State.SPACING);
        //         }
        //         break;

        //     case SPACING:
        //         if (ballsShot < ballsToShoot) robot.intakeMotor.setPower(reversePower / rpmScale);

        //         if (timer.seconds() > adjSpacing) {
        //             robot.intakeMotor.setPower(0);
        //             transitionTo(State.RECOVERING);
        //         }
        //         break;

        //     case RECOVERING:
        //         double movePenalty = (Math.abs(helpers.drive) + Math.abs(helpers.strafe)) * 0.1;
        //         if (robot.shooterAtSpeed(50) && timer.seconds() > minRecoverTime + movePenalty) {
        //             if (ballsShot < ballsToShoot) transitionTo(State.RAISING_RAMP);
        //             else transitionTo(State.IDLE);
        //         }
        //         break;
        // }
    }

    private void transitionTo(State newState) {
        currentState = newState;
        timer.reset();
    }

    public boolean isBusy() { return currentState != State.IDLE; }
}
