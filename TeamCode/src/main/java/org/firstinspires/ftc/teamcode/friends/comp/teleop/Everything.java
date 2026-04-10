package org.firstinspires.ftc.teamcode.friends.comp.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.controllers.GoalEstimate;
import org.firstinspires.ftc.teamcode.friends.controllers.GoalFusion;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.friends.controllers.TurretController;

@TeleOp(name = "FSM: Drive + Fusion + Shooting")
public class Everything extends LinearOpMode {
    private RobotHardware robotHardware;
    private Robot robot;

    private TurretController turretController;
    private ShooterController shooterController;
    private GoalFusion goalFusion;

    private boolean hasReachedRPM = false;
    private boolean lastWantsTracking = false;

    enum RobotState { IDLE, INTAKING, OUTTAKING, SHOOTING }
    private RobotState currentState = RobotState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);

        turretController = new TurretController();
        shooterController = new ShooterController();
        goalFusion = new GoalFusion();

        telemetry.addLine("WARNING: Ensure the turret is pointing forward");
        waitForStart();

        robot.shooter.startLimelight(false);
        robot.turret.resetEncoder();
        robotHardware.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            // Gather data
            LLResult llResult = robotHardware.limelight.getLatestResult();
            robot.pinpointDriver.update();
            Pose2D pose = robot.pinpointDriver.getPosition();
            double turretAngle = robot.turret.getAngle();
            double currentRPM = robot.shooter.getRPM();

            // Always calculate the goal's position in the background
            GoalEstimate estimate = goalFusion.update(llResult, pose, turretAngle);

            // SHOOTER LOGIC

            // Default values
            double targetRPM = RobotConstants.Shooter.IDLE_RPM;
            double hoodPos = 0.0;

            if (gamepad2.dpad_up) {
                targetRPM = RobotConstants.Shooter.FAR_RPM;
                hoodPos = 0.20;
            } else if (gamepad2.dpad_down) {
                targetRPM = RobotConstants.Shooter.CLOSE_RPM;
                hoodPos = 0.0;
            }

            robot.shooter.setHoodPosition(hoodPos);
            double shooterPower = shooterController.update(targetRPM, currentRPM);
            robot.shooter.setPower(shooterPower);

            // TURRET LOGIC

            boolean wantsTracking = gamepad2.triangle && estimate.isValid;

            if (wantsTracking != lastWantsTracking) {
                turretController.reset();
            }
            lastWantsTracking = wantsTracking;

            double turretPower;
            if (wantsTracking) { // Tracking
                turretPower = turretController.update(estimate.degreesFromTarget);

                if (turretController.isAligned()) {
                    gamepad2.rumble(100);
                }
            } else if (gamepad2.cross) { // Homing
                turretPower = turretController.update(turretAngle);
            } else { // Braking
                turretPower = 0.0;
            }

            robot.turret.setPower(turretPower);

            // INTAKE LOGIC (with shooter logic)

            if (gamepad2.dpad_down || gamepad2.dpad_up) {
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
                        robot.intake.intake();
                    }
                    else {
                        robot.intake.stop();
                    }
                    break;
            }

            // Rumble to warn the driver if the intake has jammed
            if (robot.intake.getCurrent(CurrentUnit.AMPS) > 3.75) {
                gamepad1.rumble(100);
            }

            // Write to hardware
            robot.mecanumDrive.move(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            telemetry.addData("RPM", currentRPM);
            telemetry.addData("Pose", pose);

            telemetry.addData("Angle", turretAngle);
            telemetry.addData("Goal angle", estimate.degreesFromTarget);
            telemetry.addData("Goal distance", estimate.distance);
            telemetry.addData("Turret power", turretPower);

            telemetry.addData("Current draw", robot.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
