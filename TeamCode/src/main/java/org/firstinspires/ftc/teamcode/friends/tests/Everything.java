package org.firstinspires.ftc.teamcode.friends.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.friends.comp.Helpers;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.friends.vision.TagPoseEstimator;
import org.firstinspires.ftc.teamcode.friends.tests.OdometryShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {

    hardwareMap robot;
    Helpers comp;
    VisionAlign vision;
    ShooterController AutoShoot;
    Limelight3A limelight;
    Follower follower;
    GoBildaPinpointDriver pinpoint;


    boolean AutoDriveActive = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- HARDWARE INIT ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        robot = new hardwareMap(hardwareMap);
        comp = new Helpers(robot);
        vision = new VisionAlign();
        AutoShoot = new ShooterController();

        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        // --- ODOMETRY / PEDRO ---
        boolean IsBlue = false; // set before match
        boolean close = true;   // set based on auto

        follower = Constants.createFollower(hardwareMap);
        AutoDrive autoDrive = new AutoDrive(follower, IsBlue, close);
        follower.setStartingPose(autoDrive.getAutoParkingPose());

        // Odometry shooter: only computes tag→goal offset and distance
        OdometryShooter odometryShooter = new OdometryShooter();

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // INPUTS
            // =========================
            comp.updateGamepads(gamepad1, gamepad2);
            comp.readDriveInputs();

            telemetry.addData("Raw Forward", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Raw Strafe", pinpoint.getPosX((DistanceUnit.INCH)));

            // =========================
            // ODOMETRY
            // =========================
            follower.update();
            Pose robotPose = follower.getPose();

            // =========================
            // VISION
            // =========================
            LLResult result = limelight.getLatestResult();

            // 1) Compute tag pose from robot + vision
            Pose tagPose = TagPoseEstimator.computeTagPose(robotPose, result);

            // 2) Compute goal pose from tag pose (6 right, 2 up)
            Pose goalPose = TagPoseEstimator.computeGoalPoseFromTag(tagPose);

            // 3) Compute small angular offset tag → goal
            double offsetDeg = odometryShooter.getTagToGoalOffset(robotPose, tagPose, goalPose);

            // 4) Vision-driven turret with odometry offset
            vision.update(
                    result,
                    true,
                    robot.turretMotor.getCurrentPosition()
            );
            robot.turretMotor.setPower(vision.turretRotatePower);

            // =========================
            // DISTANCE + SHOOTER
            // =========================
            // distance = distance from ROBOT to GOAL (not tag)
            double distance = odometryShooter.getDistanceToGoal(robotPose, goalPose);

            double targetRPM = odometryShooter.getTargetRPM(
                    distance,
                    3300, 4100,   // min/max RPM
                    60, 130       // min/max distance (inches) to map over
            );

            double hoodPos = odometryShooter.getHoodPosition(
                    distance,
                    0.00, 0.25,   // min/max hood position
                    60, 130       // same distance range
            );

            // Smooth RPM target
            robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;

            // =========================
            // AUTO DRIVE TRIGGERS
            // =========================
            if (comp.currentGp1.right_bumper && !comp.previousGp1.right_bumper) {
                if (distance > 5) {
                    autoDrive = new AutoDrive(follower, IsBlue, true);
                    autoDrive.driveToShoot();
                    AutoDriveActive = true;
                }
            }

            if (comp.currentGp1.left_bumper && !comp.previousGp1.left_bumper) {
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
            // AUTO SHOOT LOGIC
            // =========================
            boolean readyToShoot =
                    vision.isAligned &&
                            Math.abs(comp.rotate) < 0.1 &&
                            Math.abs(comp.drive) < 0.1 &&
                            Math.abs(comp.strafe) < 0.1 &&
                            Math.abs(vision.turretRotatePower) < 0.03 &&
                            gamepad1.a;

            if (!AutoShoot.isBusy() && readyToShoot) {
                AutoShoot.startShooting(3, hoodPos, targetRPM);
            }

            // =========================
            // DRIVE CONTROL
            // =========================
            if (!AutoDriveActive && !AutoShoot.isBusy()) {
                comp.applyDrive();
            }

            // =========================
            // SHOOTER STATE MACHINE
            // =========================
            AutoShoot.update(robot, comp, vision);

            if(!AutoShoot.isBusy()){
                // Intake
                if (gamepad2.right_trigger > 0.1) {
                    robot.intakeMotor.setPower(1.0);
                } else if (gamepad2.left_trigger > 0.1) {
                    robot.intakeMotor.setPower(-1.0);
                } else {
                    robot.intakeMotor.setPower(0);
                }
            }

            // =========================
            // TELEMETRY
            // =========================
            telemetry.addData("Robot Pose", robotPose);
            telemetry.addData("Distance to Goal (in)", distance);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("tx",  result.getTx());
            telemetry.addData("Hood", hoodPos);
            telemetry.addData("Shooter RPM", robot.getShooterRPM());
            telemetry.addData("TurretPower", vision.turretRotatePower);
            telemetry.addData("Turret ticks", robot.turretMotor.getCurrentPosition());
            telemetry.addData("Turret aligned", vision.isAligned);
            telemetry.addData("EstDist", distance);
            telemetry.update();
        }
    }
}