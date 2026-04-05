package org.firstinspires.ftc.teamcode.friends.comp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.friends.tests.AutoDrive;
import org.firstinspires.ftc.teamcode.friends.tests.ShooterController;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.friends.tests.OdometryShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {

    Robot robot;
    Helpers helpers;
    VisionAlign vision;
    ShooterController AutoShoot;
    Limelight3A limelight;
    Follower follower;
    GoBildaPinpointDriver pinpoint;
    OdometryShooter odometryShooter;

    boolean AutoDriveActive = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- HARDWARE INIT ---


        robot = new Robot(hardwareMap);
        pinpoint = robot.pinpoint;
        helpers = new Helpers(robot);
        vision = new VisionAlign();
        AutoShoot = new ShooterController();
        odometryShooter = new OdometryShooter();

        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = robot.limelight;
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1); //0 blue 1 red

        // --- ODOMETRY / PEDRO ---
        boolean IsBlue = false; // set before match
        boolean close = true;   // set based on auto

        follower = Constants.createFollower(hardwareMap);
        AutoDrive autoDrive = new AutoDrive(follower, IsBlue, close);
        follower.setStartingPose(autoDrive.getAutoParkingPose());

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // INPUTS
            // =========================
            helpers.updateGamepads(gamepad1, gamepad2);
            helpers.readDriveInputs();
            // =========================
            // DRIVE CONTROL
            // =========================
            helpers.applyDrive();

            // =========================
            // ODOMETRY
            // =========================
            follower.update();
            Pose robotPose = follower.getPose();
            Pose goalPose = autoDrive.getGoalPose();

            // =========================
            // VISION
            // =========================
            LLResult result = limelight.getLatestResult();

            // 4) Vision-driven turret with odometry offset
            vision.update(
                    result,
                    true,
                    robot.turretMotor.getCurrentPosition(),
                    robotPose,
                    goalPose
            );
            if(gamepad2.y)
            {
                robot.turretMotor.setPower(vision.turretRotatePower);
            }

            // =========================
            // DISTANCE + SHOOTER
            // =========================
            // distance = distance from ROBOT to GOAL (not tag)
            AutoShoot.update(robot, vision, helpers, robotPose, goalPose); // Looking kinda clean now

            // =========================
            // DRIVER SHOOTER TRIGGER
            // =========================
            if (gamepad2.a && !AutoShoot.isBusy()) {
                AutoShoot.startShooting(3); // hood/RPM ignored now
            }


            // =========================
            // AUTO DRIVE TRIGGERS
            // =========================

            double distance = odometryShooter.getDistanceToGoal(robotPose, goalPose);

            if (helpers.currentGp1.left_bumper && !helpers.previousGp1.left_bumper && !AutoDriveActive) {
                if (distance > 5) {
                    autoDrive = new AutoDrive(follower, IsBlue, true);
                    autoDrive.driveToShoot();
                    AutoDriveActive = true;
                }
            }

            if (helpers.currentGp1.left_bumper && !helpers.previousGp1.left_bumper && !AutoDriveActive) {
                if (distance > 5) {
                    autoDrive = new AutoDrive(follower, IsBlue, false);
                    autoDrive.driveToShoot();
                    AutoDriveActive = true;
                }
            }

            if (AutoDriveActive && !follower.isBusy()) {
                AutoDriveActive = false;
            }


            // =========================
            // SHOOTER STATE MACHINE
            // =========================

            if (!AutoShoot.isBusy()) {
                // Intake
                if (gamepad1.right_trigger > 0.1) {
                    robot.intakeMotor.setPower(1.0);
                } else if (gamepad1.left_trigger > 0.1) {
                    robot.intakeMotor.setPower(-1.0);
                } else {
                    robot.intakeMotor.setPower(0);
                }
            }

            // =========================
            // TELEMETRY
            // =========================
            telemetry.addLine("----ODOMETRY----");
            telemetry.addLine();
            telemetry.addData("Raw Forward", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Raw Strafe", pinpoint.getPosX((DistanceUnit.INCH)));
            telemetry.addData("Robot Pose", robotPose);
            telemetry.addData("Distance to Goal (in)", distance);
            telemetry.addLine();
            telemetry.addLine("----SHOOTER----");
            telemetry.addLine();
            telemetry.addData("Target RPM", AutoShoot.targetRPM);
            telemetry.addData("Shooter RPM", robot.getShooterRPM());
            telemetry.addData("Hood", AutoShoot.hoodPos);
            telemetry.addLine();
            telemetry.addLine("----VISION----");
            telemetry.addLine();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("TurretPower", vision.turretRotatePower);
            telemetry.addData("Turret ticks", robot.turretMotor.getCurrentPosition());
            telemetry.addData("Turret aligned", vision.isAligned);
            telemetry.addData("Turret currentAngle", vision.currentTurretAngle);
            telemetry.addLine();
            telemetry.update();
        }

    }
}