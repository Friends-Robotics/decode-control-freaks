package org.firstinspires.ftc.teamcode.friends.comp;

import com.pedropathing.geometry.BezierLine;
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

import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.tests.AutoDrive;
import org.firstinspires.ftc.teamcode.friends.tests.OdometryShooter;
import org.firstinspires.ftc.teamcode.friends.vision.TagPoseEstimator;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.friends.tests.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FullAutoAny")
public class FullAutoAny extends LinearOpMode {

    // ---------- Hardware ----------
    hardwareMap robot;
    Follower follower;
    Limelight3A limelight;
    VisionAlign vision;
    Helpers comp;
    ShooterController shooterController;
    OdometryShooter odometryShooter;
    AutoDrive autoDrive;

    // ---------- Autonomous states ----------
    enum AutoState {
        CHECK_VISION,
        VISION_ALIGN,
        SHOOTING_CYCLE,
        DRIVE_TO_INTAKE,
        DRIVE_TO_SHOOT,
        PARKING,
        DONE
    }

    AutoState currentState = AutoState.CHECK_VISION;
    ElapsedTime stateTimer = new ElapsedTime();

    // ---------- Poses ----------
    Pose startPose;
    Pose shootPose;
    Pose parkPose;

    //Determine poses
    double AutoForRedIntake1Offset; // Adds to the x value for Auto for blue
    //Middle of field x value = 72;
    double AutoForRedIntake2Offset;
    boolean blue = false; // set before match;
    boolean Close = true; // set before match
    boolean PosesMirrored = false;

    Pose[] intakePoses = {
            new Pose(42, 84, Math.toRadians(180)),
            new Pose(42, 60, Math.toRadians(180)),
            new Pose(42, 36, Math.toRadians(180)),
    };
    Pose[] intakePoses2 = {
            new Pose(21, 84, Math.toRadians(180)),
            new Pose(9, 60, Math.toRadians(180)),
            new Pose(9, 36, Math.toRadians(180)),
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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        comp = new Helpers(robot);
        vision = new VisionAlign();
        shooterController = new ShooterController();
        odometryShooter = new OdometryShooter();
        autoDrive = new AutoDrive(follower,blue,Close);
        shootPose = autoDrive.getShootPose();
        startPose = autoDrive.getShootPose();
        parkPose = autoDrive.getAutoParkingPose();// same
        follower.setStartingPose(startPose);


        robot.turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        stateTimer.reset();
        buildNewCycle();

        // ---------- Main loop ----------
        while (opModeIsActive()) {

            // =========================
// ODOMETRY UPDATE
// =========================
            follower.update();
            Pose robotPose = follower.getPose();

// =========================
// VISION
// =========================
            LLResult result = limelight.getLatestResult();

// 1) Compute REAL tag pose from robot + vision
            Pose tagPose = TagPoseEstimator.computeTagPose(robotPose, result);

// 2) Compute goal pose from tag pose (6 right, 2 up)
            Pose goalPose = TagPoseEstimator.computeGoalPoseFromTag(tagPose);


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
// distance = distance from ROBOT to GOAL (not tag, not shootPose)
            double distance = odometryShooter.getDistanceToGoal(robotPose, goalPose);

            double targetRPM = odometryShooter.getTargetRPM(
                    distance,
                    3300, 4100,
                    60, 130
            );

            double hoodPos = odometryShooter.getHoodPosition(
                    distance,
                    0.00, 0.25,
                    60, 130
            );

// Smooth RPM
            robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;

// Shooter state machine
            shooterController.update(robot, comp, vision);

            switch (currentState) {


                case CHECK_VISION:
                    if(Close)
                    {
                        currentState = AutoState.VISION_ALIGN;
                    }
                    else{
                        currentState = AutoState.SHOOTING_CYCLE;
                    }

                case VISION_ALIGN:
                    if (!follower.isBusy()) {
                        double drive;
                        if(Close){
                            drive = vision.drivePowerClose;
                        }
                        else
                        {
                            drive = vision.drivePowerFar;
                        }
                        if (stateTimer.seconds() > 3.0) {
                            // proceed anyway or skip cycle
                            currentState = AutoState.SHOOTING_CYCLE;
                        }
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
                        stateTimer.reset();
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
                        currentState = AutoState.CHECK_VISION; // align and shoot next cycle
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
                    if(!follower.isBusy())
                    {
                        robot.stopShooter();
                        robot.stopIntake();
                    }
                    break;
            }

            // ---------- Telemetry ----------
            telemetry.addData("State", currentState);
            telemetry.addData("Cycle", cycleIndex);
            telemetry.addData("X", robotPose.getX());
            telemetry.addData("Y", robotPose.getY());
            telemetry.addData("Heading", Math.toRadians(robotPose.getHeading()));
            telemetry.update();
        }
    }

    /* ---------- Build Paths ---------- */
    private void buildNewCycle() {
        Pose currentPose = follower.getPose();
        if (cycleIndex >= intakePoses.length) return;

        if (!blue && !PosesMirrored) {
            PosesMirrored = true;
            for (int i = 0; i < intakePoses.length; i++) {
                AutoForRedIntake1Offset = (72 - intakePoses[i].getX()) * 2;
                AutoForRedIntake2Offset = (72 - intakePoses2[i].getX()) * 2;
                intakePoses[i] = new Pose(
                        intakePoses[i].getX() + AutoForRedIntake1Offset,
                        intakePoses[i].getY(),
                        intakePoses[i].getHeading() + Math.toRadians(180)
                );
                intakePoses2[i] = new Pose(
                        intakePoses2[i].getX() + AutoForRedIntake2Offset,
                        intakePoses2[i].getY(),
                        intakePoses2[i].getHeading() + Math.toRadians(180)
                );
            }
        }

        Pose intakePose  = intakePoses[cycleIndex];
        Pose intakePose2 = intakePoses2[cycleIndex];

        intakeFullPath = new PathBuilder(follower)
                .addPath(new Path(new BezierLine(currentPose, intakePose)))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        intakePose.getHeading()
                )
                .addPath(new Path(new BezierLine(intakePose, intakePose2)))
                .setLinearHeadingInterpolation(
                        intakePose.getHeading(),
                        intakePose2.getHeading()
                )
                .build();

        shootPath = new PathBuilder(follower)
                .addPath(new Path(new BezierLine(intakePose2, shootPose)))
                .setLinearHeadingInterpolation(
                        intakePose2.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        ParkPath = new PathBuilder(follower)
                .addPath(new Path(new BezierLine(currentPose, parkPose)))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        parkPose.getHeading()
                )
                .build();
    }
}