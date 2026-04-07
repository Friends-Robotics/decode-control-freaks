package org.firstinspires.ftc.teamcode.friends.comp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {
    private RobotHardware robotHardware;
    private Robot robot;

    private VisionAlign vision;
    private boolean wasAligned = false;

    private ShooterController shooterController;

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
        shooterController = new ShooterController();
        // odometryShooter = new OdometryShooter();

        telemetry.addLine("WARNING: Ensure the turret is centered");
        telemetry.update();

        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.start();
        robotHardware.limelight.pipelineSwitch(1); // 0 blue 1 red

        // // --- ODOMETRY / PEDRO ---
        // boolean IsBlue = false; // set before match
        // boolean close = true;   // set based on auto

        // follower = Constants.createFollower(hardwareMap);
        // AutoDrive autoDrive = new AutoDrive(follower, IsBlue, close);
        // follower.setStartingPose(autoDrive.getAutoParkingPose());

        waitForStart();

        robot.stopFeed();
        robot.turret.resetEncoder();

        while (opModeIsActive()) {
            // Inputs
            updateGamepads();

            // ------ Gamepad One ------

            // Driving
            robot.mecanumDrive.move(
                    currentGp1.left_stick_y * RobotConstants.Drive.SPEED_MULTIPLIER,
                    -currentGp1.left_stick_x * RobotConstants.Drive.SPEED_MULTIPLIER,
                    -currentGp1.right_stick_x * RobotConstants.Drive.SPEED_MULTIPLIER
            );

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

            // Odometry
            // follower.update();
            // Pose robotPose = follower.getPose();
            // Pose goalPose = autoDrive.getGoalPose();

            // Vision
            LLResult result = robotHardware.limelight.getLatestResult();

            // Vision-driven turret
            boolean doTracking = currentGp2.y;

            vision.update(
                    result,
                    doTracking,
                    robot.turret.getAngle()
            );
            robotHardware.turretMotor.setPower(vision.getOutputPower());

            if (doTracking && vision.isNewTargetAcquired()) {
                // Jolt when a new tag is aquired
                currentGp2.rumble(0.3, 0.3, 300);
                wasAligned = false;
            } else if (doTracking && vision.isAligned()) {
                // Rumble gently when aligned
                if (!wasAligned) {
                    currentGp2.rumble(0.1, 0.1, 100000);
                    wasAligned = true;
                }
            } else {
                currentGp2.stopRumble();
                wasAligned = false;
            }

            // =========================
            // DISTANCE + SHOOTER
            // =========================
            // distance = distance from ROBOT to GOAL (not tag)
            // AutoShoot.update(robotHardware, vision, helpers, robotPose, goalPose); // Looking kinda clean now

            shooterController.update(robot, gamepad2.left_trigger > 0.3, 1600, 0);

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