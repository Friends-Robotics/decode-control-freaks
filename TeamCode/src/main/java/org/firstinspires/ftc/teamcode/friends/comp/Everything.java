package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

    enum RobotState { IDLE, INTAKING, OUTTAKING, ALIGNING, SHOOTING }
    private RobotState currentState = RobotState.IDLE;
    private RobotState lastState = RobotState.IDLE;

    private final Gamepad currentGp1 = new Gamepad();
    private final Gamepad currentGp2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);

        turretController = new TurretController();
        shooterController = new ShooterController();
        goalFusion = new GoalFusion();

        waitForStart();
        robot.shooter.startLimelight(false);

        while (opModeIsActive()) {
            currentGp1.copy(gamepad1);
            currentGp2.copy(gamepad2);

            // Gather data
            LLResult llResult = robotHardware.limelight.getLatestResult();
            Pose2D pose = robotHardware.pinpoint.getPosition();
            double turretAngle = robot.turret.getAngle();
            double currentRPM = robot.shooter.getRPM();

            // Update the estimate of the goal
            GoalEstimate estimate = goalFusion.update(llResult, pose, turretAngle);

            // 2. STATE TRANSITION LOGIC
            if (currentGp2.dpad_up || currentGp2.dpad_down) {
                double target = currentGp2.dpad_up ? RobotConstants.Shooter.FAR_RPM : RobotConstants.Shooter.CLOSE_RPM;

                // Switch to SHOOTING only if Shooter is at RPM AND Turret is pointed at target
                if (shooterController.isReady(target, currentRPM) && turretController.isAligned(estimate.error)) {
                    currentState = RobotState.SHOOTING;
                } else {
                    currentState = RobotState.ALIGNING;
                }
            } else if (currentGp1.right_trigger > 0.1) {
                currentState = RobotState.INTAKING;
            } else if (currentGp1.left_trigger > 0.1) {
                currentState = RobotState.OUTTAKING;
            } else {
                currentState = RobotState.IDLE;
            }

            // 3. RESET ON TRANSITION
            if (currentState != lastState) {
                shooterController.reset();
                turretController.reset();
            }
            lastState = currentState;

            // 4. EXECUTION
            double turretPower = 0;
            double shooterPower = 0;

            switch (currentState) {
                case IDLE:
                    robot.intake.stop();
                    robot.shooter.stopFeed();
                    // Home to 0 degrees using neutral distance for PID gains
                    turretPower = turretController.update(0.0 - turretAngle, 40.0);
                    break;

                case INTAKING:
                    robot.intake.intake();
                    robot.shooter.stopFeed();
                    turretPower = turretController.update(0.0 - turretAngle, 40.0);
                    break;

                case OUTTAKING:
                    robot.intake.outtake();
                    robot.shooter.startFeed(); // Use feed to help push out
                    break;

                case ALIGNING:
                case SHOOTING:
                    double targetRPM = currentGp2.dpad_up ? RobotConstants.Shooter.FAR_RPM : RobotConstants.Shooter.CLOSE_RPM;
                    double hoodPos = currentGp2.dpad_up ? 0.25 : 0.0;

                    robot.shooter.setHoodPosition(hoodPos);
                    shooterPower = shooterController.calculate(targetRPM, currentRPM);
                    turretPower = turretController.update(estimate.error, estimate.distance);

                    if (currentState == RobotState.SHOOTING) {
                        robot.shooter.startFeed();
                        robot.intake.intake();
                    } else {
                        robot.shooter.stopFeed();
                        robot.intake.stop();
                    }
                    break;
            }

            // 5. HARDWARE WRITE
            robot.mecanumDrive.move(currentGp1.left_stick_y, -currentGp1.left_stick_x, -currentGp1.right_stick_x);
            robot.turret.setPower(turretPower);
            robot.shooter.setPower(shooterPower);

            if (robot.intake.getCurrent(CurrentUnit.AMPS) > 3.5) gamepad1.rumble(300);

            updateTelemetry(estimate, currentRPM);
        }
    }

    private void updateTelemetry(GoalEstimate est, double rpm) {
        telemetry.addData("State", currentState);
        telemetry.addData("Target Found", est.isEstimated);
        telemetry.addData("Dist", (int)est.distance + " in");
        telemetry.addData("RPM", (int)rpm);
        telemetry.update();
    }
}
