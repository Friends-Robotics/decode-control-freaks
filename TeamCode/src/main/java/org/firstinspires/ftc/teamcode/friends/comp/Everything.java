package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {
    private RobotHardware robotHardware;
    private Robot robot;

    private VisionAlign vision;
    // private Follower follower;
    // private OdometryShooter odometryShooter;

    private final Gamepad currentGp1 = new Gamepad();
    private final Gamepad previousGp1 = new Gamepad();
    private final Gamepad currentGp2 = new Gamepad();
    private final Gamepad previousGp2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);

        vision = new VisionAlign();
        // odometryShooter = new OdometryShooter();

        // telemetry.addLine("WARNING: Ensure the turret is centered");
        // robot.turret.resetEncoder();

        // robotHardware.limelight.setPollRateHz(100);
        // robotHardware.limelight.start();
        // robotHardware.limelight.pipelineSwitch(1); // 0 blue 1 red

        // // --- ODOMETRY / PEDRO ---
        // boolean IsBlue = false; // set before match
        // boolean close = true;   // set based on auto

        // follower = Constants.createFollower(hardwareMap);
        // AutoDrive autoDrive = new AutoDrive(follower, IsBlue, close);
        // follower.setStartingPose(autoDrive.getAutoParkingPose());

        robot.stopFeed();

        waitForStart();

        while (opModeIsActive()) {
            // Inputs
            updateGamepads();

            // Driving
            robot.mecanumDrive.move(
                    currentGp1.left_stick_y * RobotConstants.Drive.SPEED_MULTIPLIER,
                    -currentGp1.left_stick_x * RobotConstants.Drive.SPEED_MULTIPLIER,
                    -currentGp1.right_stick_x * RobotConstants.Drive.SPEED_MULTIPLIER
            );

            // Odometry
            // follower.update();
            // Pose robotPose = follower.getPose();
            // Pose goalPose = autoDrive.getGoalPose();

            // Vision
            LLResult result = robotHardware.limelight.getLatestResult();

            // Vision-driven turret
            vision.update(
                    result,
                    currentGp2.y,
                    robot.turret.getAngle()
            );
            robotHardware.turretMotor.setPower(vision.getOutputPower());

            // =========================
            // DISTANCE + SHOOTER
            // =========================
            // distance = distance from ROBOT to GOAL (not tag)
            // AutoShoot.update(robotHardware, vision, helpers, robotPose, goalPose); // Looking kinda clean now

            // =========================
            // DRIVER SHOOTER TRIGGER
            // =========================
            // if (gamepad2.a && !AutoShoot.isBusy()) {
            //     AutoShoot.startShooting(3); // hood/RPM ignored now
            // }


            // =========================
            // AUTO DRIVE TRIGGERS
            // =========================
            // Auto drive is a bad idea
            // double distance = odometryShooter.getDistanceToGoal(robotPose, goalPose);

            // if (helpers.currentGp1.left_bumper && !helpers.previousGp1.left_bumper && !AutoDriveActive) {
            //     if (distance > 5) {
            //         autoDrive = new AutoDrive(follower, IsBlue, true);
            //         autoDrive.driveToShoot();
            //         AutoDriveActive = true;
            //     }
            // }

            // if (helpers.currentGp1.left_bumper && !helpers.previousGp1.left_bumper && !AutoDriveActive) {
            //     if (distance > 5) {
            //         autoDrive = new AutoDrive(follower, IsBlue, false);
            //         autoDrive.driveToShoot();
            //         AutoDriveActive = true;
            //     }
            // }

            // if (AutoDriveActive && !follower.isBusy()) {
            //     AutoDriveActive = false;
            // }


            // =========================
            // SHOOTER STATE MACHINE
            // =========================

            // if (!AutoShoot.isBusy()) {
            // Intake
            if (currentGp1.right_trigger > 0.1) {
                robot.stopFeed();
                robot.intake();
            } else if (currentGp1.left_trigger > 0.1) {
                robot.startFeed();
                robot.outtake();
            } else {
                robot.stopFeed();
                robot.stopIntake();
            }

            // Telemetry
            telemetry.addLine("----ODOMETRY----");
            telemetry.addLine();
            telemetry.addData("Raw Forward", robotHardware.pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Raw Strafe", robotHardware.pinpoint.getPosX((DistanceUnit.INCH)));
            // telemetry.addData("Robot Pose", robotPose);
            // telemetry.addData("Distance to Goal (in)", distance);
            telemetry.addLine();
            telemetry.addLine("----SHOOTER----");
            telemetry.addLine();
            // telemetry.addData("Target RPM", AutoShoot.targetRPM);
            telemetry.addData("Shooter RPM", robot.getShooterRPM());
            // telemetry.addData("Hood", AutoShoot.hoodPos);
            telemetry.addLine();
            telemetry.addLine("----VISION----");
            telemetry.addLine();
            // telemetry.addData("tx", result.getTx());
            // telemetry.addData("TurretPower", vision.getOutputPower());
            telemetry.addData("Turret ticks", robotHardware.turretMotor.getCurrentPosition());
            telemetry.addData("Turret state: ", vision.getCurrentState());
            // telemetry.addData("Turret currentAngle", robot.getTurretAngle());
            telemetry.addLine();
            telemetry.update();
        }
    }

    private void updateGamepads() {
        previousGp1.copy(currentGp1);
        currentGp1.copy(gamepad1);

        previousGp2.copy(currentGp2);
        currentGp2.copy(gamepad2);
    }
}