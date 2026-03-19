package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.comp.Comp;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.tests.OdometryShooter;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.friends.tests.ShooterController;

@Autonomous(name = "AutoBlueClose")
public class AutoForBlueClose extends LinearOpMode {

    // ---------- Hardware ----------
    hardwareMap robot;
    Follower follower;
    Limelight3A limelight;
    VisionAlign vision;
    Comp comp;
    ShooterController shooterController;
    OdometryShooter odometryShooter;

    // ---------- Autonomous states ----------
    enum AutoState {
        DRIVE_TO_PRELOAD,
        VISION_ALIGN,
        SHOOTING_CYCLE,
        DRIVE_TO_INTAKE,
        DRIVE_TO_SHOOT,
        PARKING,
        DONE
    }

    AutoState currentState = AutoState.DRIVE_TO_PRELOAD;
    ElapsedTime stateTimer = new ElapsedTime();

    // ---------- Poses ----------
    Pose startPose = new Pose(16, 130, Math.toRadians(135));
    Pose shootPose = new Pose(60, 84, Math.toRadians(135));
    Pose parkPose = new Pose(60, 108, Math.toRadians(135));

    Pose[] intakePoses = {
            new Pose(35, 85, Math.toRadians(180)),
            new Pose(35, 60, Math.toRadians(180)),
            new Pose(35, 35, Math.toRadians(180)),
    };
    Pose[] intakePoses2 = {
            new Pose(8, 85, Math.toRadians(180)),
            new Pose(8, 60, Math.toRadians(180)),
            new Pose(8, 35, Math.toRadians(180)),
    };

    PathChain intakeFullPath;
    PathChain shootPath;
    PathChain StartShootPath;
    PathChain ParkPath;

    int cycleIndex = 0;
    static final int MAX_CYCLES = 3;

    @Override
    public void runOpMode() {

        // ---------- Initialize ----------
        robot = new hardwareMap(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        vision = new VisionAlign();
        shooterController = new ShooterController();
        odometryShooter = new OdometryShooter(
                0.01,    // kOdoAim
                3300, 4100,   // CLOSE_RPM, FAR_RPM
                60, 130,      // CLOSE_DIST, FAR_DIST
                0.00, 0.25    // CLOSE_HOOD, FAR_HOOD
        );

        robot.turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

        buildNewCycle();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        robot.startIntake(); // start intake if preload ball exists
        stateTimer.reset();

        // ---------- Main loop ----------
        while (opModeIsActive()) {

            follower.update(); // updates pose via Pinpoint
            Pose currentpose = follower.getPose();
            LLResult result = limelight.getLatestResult();
            int turretTicks = robot.turretMotor.getCurrentPosition();
            vision.update(result, true, turretTicks);

            // distance to goal
            double distance = odometryShooter.getDistance(currentpose, shootPose);

            // target RPM & hood position
            double targetRPM = odometryShooter.getTargetRPM(distance);
            double hoodPos = odometryShooter.getHoodPosition(distance);

            // turret power = odometry + vision + optional driver input
            double turretPower = odometryShooter.getTurretPower(
                    currentpose,
                    shootPose,
                    vision.turretRotatePower,  // vision correction
                    0, 0                        // no driver input in Auto
            );

            robot.turretMotor.setPower(turretPower);
            robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;

            // ---------- Shooter Controller update ----------
            shooterController.update(robot, comp, vision);

            switch (currentState) {

                case DRIVE_TO_PRELOAD:
                    follower.followPath(StartShootPath);
                    currentState = AutoState.VISION_ALIGN;
                    break;

                case VISION_ALIGN:
                    if (!follower.isBusy()) {
                        double drive = vision.drivePowerClose;
                        double strafe = 0;
                        double rotate = 0;

                        double fl = Range.clip(drive + strafe + rotate, -1, 1);
                        double fr = Range.clip(drive - strafe - rotate, -1, 1);
                        double bl = Range.clip(drive - strafe + rotate, -1, 1);
                        double br = Range.clip(drive + strafe - rotate, -1, 1);

                        robot.frontLeftMotor.setPower(fl);
                        robot.frontRightMotor.setPower(fr);
                        robot.backLeftMotor.setPower(bl);
                        robot.backRightMotor.setPower(br);

                        if (Math.abs(vision.turretRotatePower) < 0.05 &&
                                Math.abs(vision.drivePowerClose) < 0.05  &&
                                robot.shooterAtSpeed(50)) {

                            // Start shooting cycle for 3 balls
                            shooterController.startShooting(3, hoodPos, targetRPM);
                            currentState = AutoState.SHOOTING_CYCLE;
                        }
                    }
                    break;

                case SHOOTING_CYCLE:
                    // Wait for ShooterController to finish
                    if (!shooterController.isBusy()) {

                        // Start intake cycle
                        if (cycleIndex < MAX_CYCLES) {
                            buildNewCycle();
                            robot.startIntake();
                            follower.followPath(intakeFullPath);
                            currentState = AutoState.DRIVE_TO_INTAKE;
                        } else {
                            currentState = AutoState.PARKING;
                        }
                    }
                    break;

                case DRIVE_TO_INTAKE:
                    if (!follower.isBusy()) {
                        robot.stopIntake();
                        robot.setShooterRPM(robot.targetShooterRPM); // spin shooter while driving to shooting pos
                        follower.followPath(shootPath);
                        currentState = AutoState.DRIVE_TO_SHOOT;
                    }
                    break;

                case DRIVE_TO_SHOOT:
                    if (!follower.isBusy()) {
                        currentState = AutoState.VISION_ALIGN; // align and shoot next cycle
                        cycleIndex++;
                    }
                    break;

                case PARKING:
                    if (!follower.isBusy()) {
                        follower.followPath(ParkPath);
                        currentState = AutoState.DONE;
                    }
                    break;

                case DONE:
                    robot.stopShooter();
                    robot.stopIntake();
                    break;
            }

            // ---------- Telemetry ----------
            telemetry.addData("State", currentState);
            telemetry.addData("Cycle", cycleIndex);
            telemetry.addData("X", currentpose.getX());
            telemetry.addData("Y", currentpose.getY());
            telemetry.addData("Heading", Math.toRadians(currentpose.getHeading()));
            telemetry.update();
        }
    }

    /* ---------- Build Paths ---------- */
    private void buildNewCycle() {
        Pose currentPose = follower.getPose();
        if (cycleIndex >= intakePoses.length) return;

        Pose intakePose = intakePoses[cycleIndex];
        Pose intakePose2 = intakePoses2[cycleIndex];

        intakeFullPath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(currentPose, intakePose)))
                .addPath(new Path(new BezierCurve(intakePose, intakePose2)))
                .build();

        shootPath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(intakePose2, shootPose)))
                .build();

        StartShootPath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(startPose, shootPose)))
                .build();

        ParkPath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(currentPose, parkPose)))
                .build();
    }
}