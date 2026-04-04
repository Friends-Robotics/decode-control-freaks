package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RED Close 1")
public class AUTOredo extends OpMode {

    private org.firstinspires.ftc.teamcode.friends.hardwareMap robot;
    private org.firstinspires.ftc.teamcode.friends.comp.Helpers helpers;
    private org.firstinspires.ftc.teamcode.friends.vision.VisionAlign vision;
    private Follower follower;
    private boolean pathStarted = false;

    private DcMotorEx IntakeMotor;

    private double intakePower = 0.8;
    private double RintakePower = -0.8;

    // ================= STATES =================
    public enum PathState {
        DRIVE_STARTPOSE,
        DRIVE_SHOOTPOSE,
        HOLD_SHOOTPOSE,
        DRIVE_COLLECT1_1,
        DRIVE_COLLECT1_2,
        DRIVE_COLLECT2_1,
        DRIVE_COLLECT2_2,
        DRIVE_COLLECT2_RETURN,
        STOP
    }

    private PathState pathState;

    private ElapsedTime holdTimer = new ElapsedTime();
    private boolean holdStarted = false;

    // ================= POSES =================
    private final Pose startPose = new Pose(123.43, 123.03, Math.toRadians(36));
    private final Pose shootPose = new Pose(87.01, 82.29, Math.toRadians(47));
    private final Pose collect1_1 = new Pose(101.4462, 85.61029580838324, Math.toRadians(0));
    private final Pose collect1_2 = new Pose(127.99, 85.42473053892215, Math.toRadians(0));
    private final Pose collect2_1 = new Pose(97.65218802395209, 62.39058143712576, Math.toRadians(0));
    private final Pose collect2_2 = new Pose(127.2718, 62.1206365269461, Math.toRadians(0));

    // ================= PATHS =================
    private PathChain driveStartPose;
    private PathChain driveShootPose;
    private PathChain driveCollect1_1;
    private PathChain driveCollect1_2;
    private PathChain driveCollect2_1;
    private PathChain driveCollect2_2;
    private PathChain driveCollect2Return;

    private int cycleCount = 0;

    // ================= BUILD PATHS =================
    public void buildPaths() {

        driveStartPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1_1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect1_1.getHeading())
                .build();

        driveCollect1_1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1_1, collect1_2))
                .setLinearHeadingInterpolation(collect1_1.getHeading(), collect1_2.getHeading())
                .build();

        driveCollect1_2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1_2, shootPose))
                .setLinearHeadingInterpolation(collect1_2.getHeading(), shootPose.getHeading())
                .build();

        driveCollect2_1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect2_1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect2_1.getHeading())
                .build();

        driveCollect2_2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2_1, collect2_2))
                .setLinearHeadingInterpolation(collect2_1.getHeading(), collect2_2.getHeading())
                .build();

        // ✅ FIX: missing return path
        driveCollect2Return = follower.pathBuilder()
                .addPath(new BezierLine(collect2_2, shootPose))
                .setLinearHeadingInterpolation(collect2_2.getHeading(), shootPose.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathStarted = false;
        holdStarted = false;
    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveStartPose, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_SHOOTPOSE);
                }
                break;

            case DRIVE_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPose, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.HOLD_SHOOTPOSE);
                }
                break;

            case HOLD_SHOOTPOSE:
                if (!holdStarted) {
                    holdStarted = true;
                    holdTimer.reset();
                }

                IntakeMotor.setPower(RintakePower);
                // TODO remove the line above and replace with the shooter, in theory that should work, remember to add in all of the imports etc.

                if (holdTimer.seconds() > 1.0) {
                    holdStarted = false;

                    if (cycleCount == 0) {
                        cycleCount++;
                        setPathState(PathState.DRIVE_COLLECT1_1);
                    } else if (cycleCount == 1) {
                        cycleCount++;
                        setPathState(PathState.DRIVE_COLLECT2_1);
                    } else {
                        setPathState(PathState.STOP);
                    }
                }
                break;

            case DRIVE_COLLECT1_1:
                if (!pathStarted) {
                    follower.followPath(driveCollect1_1, false);
                    pathStarted = true;
                }
                IntakeMotor.setPower(intakePower);

                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT1_2);
                }
                break;

            case DRIVE_COLLECT1_2:
                if (!pathStarted) {
                    follower.followPath(driveCollect1_2, false);
                    pathStarted = true;
                }
                IntakeMotor.setPower(intakePower);

                if (!follower.isBusy()) {
                    setPathState(PathState.HOLD_SHOOTPOSE);
                }
                break;

            case DRIVE_COLLECT2_1:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_1, false);
                    pathStarted = true;
                }
                IntakeMotor.setPower(intakePower);

                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT2_2);
                }
                break;

            case DRIVE_COLLECT2_2:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_2, false);
                    pathStarted = true;
                }
                IntakeMotor.setPower(intakePower);

                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT2_RETURN);
                }
                break;

            case DRIVE_COLLECT2_RETURN:
                if (!pathStarted) {
                    follower.followPath(driveCollect2Return, false);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.HOLD_SHOOTPOSE);
                }
                break;

            case STOP:
                IntakeMotor.setPower(0);
                break;
        }
    }

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");

        pathState = PathState.DRIVE_STARTPOSE;

        buildPaths();

        follower.setPose(startPose);

        robot = new org.firstinspires.ftc.teamcode.friends.hardwareMap(hardwareMap);
        helpers = new org.firstinspires.ftc.teamcode.friends.comp.Helpers(robot);
        vision = new org.firstinspires.ftc.teamcode.friends.vision.VisionAlign();
    }

    @Override
    public void loop() {

        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}