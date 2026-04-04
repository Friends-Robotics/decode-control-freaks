package org.firstinspires.ftc.teamcode.friends.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.friends.comp.Helpers;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {
    hardwareMap robot;
    Helpers comp;
    VisionAlign vision;
    ShooterController autoShoot;
    Limelight3A limelight;
    Follower follower;
    GoBildaPinpointDriver pinpoint;

    boolean autoDriveActive = false;
    boolean isBlue = false;
    boolean close = true;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        initLimelight();
        resetSystems();

        runPipelineSelector(); // 🔥 pre-start selection

        waitForStart();

        // =========================
        // PATHING + SHOOTING SETUP
        // =========================
        follower = Constants.createFollower(hardwareMap);
        AutoDrive autoDrive = new AutoDrive(follower, isBlue, close);
        follower.setStartingPose(autoDrive.getAutoParkingPose());

        Pose goalPose = autoDrive.getGoalPose();
        OdometryShooter odometryShooter = new OdometryShooter(6, 2);

        // =========================
        // MAIN LOOP
        // =========================
        while (opModeIsActive()) {

            updateInputs();
            Pose robotPose = updateOdometry();

            LLResult result = limelight.getLatestResult();

            handleVisionAndTurret(result, robotPose, goalPose, odometryShooter);
            double distance = handleShooterCalculations(robotPose, goalPose, odometryShooter);

            autoDrive = handleAutoDrive(autoDrive, distance);

            handleAutoShooting(result, robotPose, goalPose, odometryShooter, distance);
            handleDriveAndIntake();
            handleManualOverride();

            autoShoot.update(robot, comp, vision);

            sendTelemetry(result, robotPose, distance);
        }
    }

    private void initHardware() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        robot = new hardwareMap(hardwareMap);
        comp = new Helpers(robot);
        vision = new VisionAlign();
        autoShoot = new ShooterController();

        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    private void resetSystems() {
        pinpoint.resetPosAndIMU();
    }

    private void runPipelineSelector() throws InterruptedException {
        boolean lastA = false;
        boolean lastB = false;

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.a && !lastA) isBlue = false;
            if (gamepad1.b && !lastB) isBlue = true;

            lastA = gamepad1.a;
            lastB = gamepad1.b;

            // Apply pipeline live
            limelight.pipelineSwitch(isBlue ? 0 : 1);

            telemetry.addLine("=== PIPELINE SELECT ===");
            telemetry.addLine("A = RED");
            telemetry.addLine("B = BLUE");
            telemetry.addData("Selected", isBlue ? "BLUE (1)" : "RED (0)");
            telemetry.update();

            sleep(50);
        }

        // Ensure correct pipeline after selection
        limelight.pipelineSwitch(isBlue ? 0 : 1);
    }

    private void updateInputs() {
        comp.updateGamepads(gamepad1, gamepad2);
        comp.readDriveInputs();
    }

    private Pose updateOdometry() {
        follower.update();
        return follower.getPose();
    }

    private void handleVisionAndTurret(LLResult result, Pose robotPose, Pose goalPose, OdometryShooter shooter) {

        double targetAngle = shooter.getTargetTurretAngle(robotPose, goalPose);

        vision.update(
                result,
                true,
                robot.turretMotor.getCurrentPosition(),
                targetAngle
        );

        robot.turretMotor.setPower(vision.turretRotatePower);
    }

    private double handleShooterCalculations(Pose robotPose, Pose goalPose, OdometryShooter shooter) {

        double distance = shooter.getDistanceToGoal(robotPose, goalPose);

        double targetRPM = shooter.getTargetRPM(distance, 3300, 4100, 60, 130);
        double hoodPos = shooter.getHoodPosition(distance, 0.00, 0.25, 60, 130);

        robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;

        return distance;
    }

    private AutoDrive handleAutoDrive(AutoDrive autoDrive, double distance) {

        if (comp.currentGp1.right_bumper && !comp.previousGp1.right_bumper) {
            if (distance > 5) {
                autoDrive = new AutoDrive(follower, isBlue, true);
                autoDrive.driveToShoot();
                autoDriveActive = true;
            }
        }

        if (comp.currentGp1.left_bumper && !comp.previousGp1.left_bumper) {
            if (distance > 5) {
                autoDrive = new AutoDrive(follower, isBlue, false);
                autoDrive.driveToShoot();
                autoDriveActive = true;
            }
        }

        if (autoDriveActive && !follower.isBusy()) {
            autoDriveActive = false;
        }

        return autoDrive;
    }

    private void handleAutoShooting(LLResult result, Pose robotPose, Pose goalPose,
                                    OdometryShooter shooter, double distance) {

        double targetRPM = shooter.getTargetRPM(distance, 3300, 4100, 60, 130);
        double hoodPos = shooter.getHoodPosition(distance, 0.00, 0.25, 60, 130);

        boolean readyToShoot =
                vision.isAligned &&
                        Math.abs(comp.rotate) < 0.1 &&
                        Math.abs(comp.drive) < 0.1 &&
                        Math.abs(comp.strafe) < 0.1 &&
                        Math.abs(vision.turretRotatePower) < 0.03 &&
                        gamepad1.a;

        if (!autoShoot.isBusy() && readyToShoot) {
            autoShoot.startShooting(3, hoodPos, targetRPM);
        }
    }

    private void handleDriveAndIntake() {
        if (!autoDriveActive && !autoShoot.isBusy()) {
            comp.applyDrive();
            comp.handleIntake();
        }
    }

    private void handleManualOverride() {
        boolean manualOverride = gamepad2.dpad_right;

        if (manualOverride) {
            robot.hood.setPosition(0);
            comp.handleTurret();
            comp.handleShooterAngle();
        } else {
            robot.turretMotor.setPower(vision.turretRotatePower);
        }
    }

    private void sendTelemetry(LLResult result, Pose robotPose, double distance) {
        telemetry.addData("Distance", distance);
        telemetry.addData("Shooter RPM", robot.getShooterRPM());
        telemetry.addData("Turret Power", robot.turretMotor.getPower());
        telemetry.addData("Drive Active", autoDriveActive);
        telemetry.addData("Vision Ta", result != null ? result.getTa() : 0);
        telemetry.addData("Aligned", vision.isAligned);
        telemetry.addData("Pose", robotPose);
        telemetry.update();
    }
}