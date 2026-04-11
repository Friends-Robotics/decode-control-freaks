package org.firstinspires.ftc.teamcode.friends.comp.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.controllers.*;
import org.firstinspires.ftc.teamcode.friends.helpers.Utils;

@TeleOp(name = "Everything", group = "TeleOp")
public class Everything extends LinearOpMode {

    private boolean hasReachedRPM = false;
    private boolean lastWantsTracking = false;
    private boolean lastWasHoming = false;
    private double latchedDistance = 0.0;
    private double latchedAngle = 0.0;
    private double startingLatchAngle = 0.0;
    private boolean isLatchedAngle = false;

    enum RobotState { IDLE, INTAKING, OUTTAKING, SHOOTING }
    private RobotState currentState = RobotState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robotHardware = new RobotHardware(hardwareMap);
        Robot robot = new Robot(robotHardware);

        TurretController turretController = new TurretController();
        ShooterController shooterController = new ShooterController();
        GoalFusion goalFusion = new GoalFusion();

        boolean isBlue = selectAlliance();

        waitForStart();

        robot.shooter.startLimelight(isBlue);
        robot.turret.resetEncoder();

        while (opModeIsActive()) {
            LLResult llResult = robot.shooter.getLimelightResult();
            robot.pinpointDriver.update();
            Pose2D pose = robot.pinpointDriver.getPosition();
            double turretAngle = robot.turret.getAngle();
            double currentRPM = robot.shooter.getRPM();

            GoalEstimate estimate = goalFusion.update(llResult, pose, turretAngle);

            double targetRPM = handleShooter(robot, shooterController, estimate, currentRPM);
            double turretPower = handleTurret(robot, turretController, estimate, turretAngle);

            handleState(robot, shooterController);
            handleDrive(robot);
            handleTelemetry(robot, estimate, currentRPM, targetRPM, turretAngle, turretPower);
        }
    }

    private boolean selectAlliance() {
        boolean isBlue = true;

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("WARNING: Ensure the turret is pointing forward");
            telemetry.addLine();
            telemetry.addLine("Which net are you shooting into");
            telemetry.addLine("Red: (Dpad Up) Blue: (Dpad Down)");

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                isBlue = false;
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                isBlue = true;
            }

            telemetry.addData("Selected", isBlue ? "Blue" : "Red");
            telemetry.update();
            sleep(50);
        }

        return isBlue;
    }

    private double handleShooter(Robot robot, ShooterController shooterController,
                                 GoalEstimate estimate, double currentRPM) {

        double targetRPM = RobotConstants.Shooter.IDLE_RPM;
        double hoodPos = 0.0;

        if (gamepad2.dpad_up) {
            targetRPM = RobotConstants.Shooter.FAR_RPM;
            hoodPos = RobotConstants.Shooter.FAR_HOOD;

        } else if (gamepad2.dpad_down) {
            targetRPM = RobotConstants.Shooter.CLOSE_RPM;
            hoodPos = RobotConstants.Shooter.CLOSE_HOOD;

        } else if (gamepad2.right_trigger > 0.1 && estimate.isValid) {
            if (latchedDistance == 0) {
                latchedDistance = estimate.distance;
            }

            targetRPM = shooterController.getInterpolatedRPM(latchedDistance);
            hoodPos = shooterController.getInterpolatedHood(latchedDistance);

        } else {
            latchedDistance = 0;
        }

        robot.shooter.setHoodPosition(hoodPos);

        double power = shooterController.update(targetRPM, currentRPM);
        robot.shooter.setPower(power);

        return targetRPM;
    }

    private double handleTurret(Robot robot, TurretController turretController,
                                GoalEstimate estimate, double turretAngle) {
        boolean wantsTracking = gamepad2.triangle && estimate.isValid;
        boolean wantsHoming = gamepad2.cross;

        if (wantsTracking != lastWantsTracking || wantsHoming != lastWasHoming) {
            turretController.reset();
        }

        lastWantsTracking = wantsTracking;
        lastWasHoming = wantsHoming;

        double power;

        if (wantsTracking) {
            if (!isLatchedAngle) {
                latchedAngle = estimate.degreesFromTarget;
                startingLatchAngle = turretAngle;
                isLatchedAngle = true;
            }

            double remainingError = latchedAngle - (turretAngle - startingLatchAngle);
            power = turretController.update(remainingError);

            if (turretController.isAligned()) {
                gamepad2.rumble(100);
            }

        } else if (wantsHoming) {
            isLatchedAngle = false;
            power = turretController.update(turretAngle);

        } else {
            isLatchedAngle = false;
            power = 0.0;
        }

        robot.turret.setPower(power);
        return power;
    }

    private void handleState(Robot robot, ShooterController shooterController) {
        if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.right_trigger > 0.1) {
            currentState = RobotState.SHOOTING;
        } else if (gamepad1.right_trigger > 0.1) {
            currentState = RobotState.INTAKING;
        } else if (gamepad1.left_trigger > 0.1) {
            currentState = RobotState.OUTTAKING;
        } else {
            currentState = RobotState.IDLE;
        }

        switch (currentState) {
            case IDLE:
                robot.intake.stop();
                robot.shooter.stopFeed();
                hasReachedRPM = false;
                break;

            case INTAKING:
                robot.intake.intake();
                robot.shooter.stopFeed();
                hasReachedRPM = false;
                break;

            case OUTTAKING:
                robot.intake.outtake();
                robot.shooter.feed();
                hasReachedRPM = false;
                break;

            case SHOOTING:
                robot.shooter.feed();

                if (shooterController.isReady()) {
                    hasReachedRPM = true;
                }

                if (hasReachedRPM) {
                    double t = Utils.getT(RobotConstants.Shooter.CLOSE_PULSE_DISTANCE, RobotConstants.Shooter.FAR_PULSE_DISTANCE, latchedDistance);
                    double offTime = Utils.lerp(0.0, RobotConstants.Shooter.FAR_PULSE_OFF, t);

                    robot.intake.pulse(RobotConstants.Shooter.PULSE_ON, offTime);
                } else {
                    robot.intake.stop();
                }
                break;
        }

        if (robot.intake.getCurrent(CurrentUnit.AMPS) > 3.75) {
            gamepad1.rumble(100);
        }
    }

    private void handleDrive(Robot robot) {
        robot.mecanumDrive.move(
                gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
    }

    private void handleTelemetry(Robot robot, GoalEstimate estimate, double currentRPM,
                                 double targetRPM,  double turretAngle, double turretPower) {
        telemetry.addData("RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);

        telemetry.addData("Angle", turretAngle);
        telemetry.addData("Goal angle", estimate.degreesFromTarget);
        telemetry.addData("Goal distance", estimate.distance);
        telemetry.addData("Turret power", turretPower);

        telemetry.addData("Current draw", robot.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}