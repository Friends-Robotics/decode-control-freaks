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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.tests.AutoDrive;
import org.firstinspires.ftc.teamcode.friends.tests.OdometryShooter;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.friends.tests.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FullAutoAny")
public class FullAutoAny extends LinearOpMode {

    // ---------- Hardware ----------
    hardwareMap robot;
    Follower follower;
    VisionAlign vision;
    Helpers comp;
    ShooterController shooterController;
    OdometryShooter odometryShooter;
    AutoDrive autoDrive;

    // ---------- Autonomous states ----------
    enum AutoState {
        SHOOTING_CYCLE,
        DRIVE_TO_INTAKE,
        DRIVE_TO_SHOOT,
        PARKING,
        DONE
    }

    AutoState currentState = AutoState.SHOOTING_CYCLE;
    ElapsedTime stateTimer = new ElapsedTime();

    // ---------- Poses ----------
    Pose startPose;
    Pose shootPose;
    Pose parkPose;
    Pose goalPose;

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
    PathChain ParkPath;

    int cycleIndex = 0;
    static final int MAX_CYCLES = 3;
    @Override
    public void runOpMode() {

        // ---------- Initialize ----------
        robot = new hardwareMap(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        comp = new Helpers(robot);
        shooterController = new ShooterController();
        odometryShooter = new OdometryShooter();
        autoDrive = new AutoDrive(follower,blue,Close);
        shootPose = autoDrive.getShootPose();
        startPose = autoDrive.getShootPose();
        parkPose = autoDrive.getAutoParkingPose();// same
        goalPose = autoDrive.getGoalPose();
        follower.setStartingPose(startPose);

        robot.turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setTargetPosition(0);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(0.3);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        stateTimer.reset();
        buildNewCycle();

        // ---------- Main loop ----------
        while (opModeIsActive()) {
            Pose currentPose = follower.getPose();

            // Shooter state machine
            shooterController.update(robot, null, comp, currentPose, goalPose);

            switch (currentState) {

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
                        follower.followPath(shootPath);
                        currentState = AutoState.DRIVE_TO_SHOOT;
                    }
                    break;

                case DRIVE_TO_SHOOT:
                    if (!follower.isBusy()) {
                        shooterController.startShooting(3);
                        currentState = AutoState.SHOOTING_CYCLE;
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
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", Math.toRadians(currentPose.getHeading()));
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