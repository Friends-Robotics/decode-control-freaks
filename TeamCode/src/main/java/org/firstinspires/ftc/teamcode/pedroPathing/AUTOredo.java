package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.friends.tests.ShooterController;


@Autonomous(name = "Drive Only Auto FIXED")
public class AUTOredo extends OpMode {

    private ShooterController shooterController;
    private org.firstinspires.ftc.teamcode.friends.hardwareMap robot;
    private org.firstinspires.ftc.teamcode.friends.comp.Helpers helpers;
    private org.firstinspires.ftc.teamcode.friends.vision.VisionAlign vision;
    private Follower follower;
    private boolean pathStarted = false;

    private DcMotor intakeMotor;

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
        STOP
    }

    private PathState pathState;

    // ================= TIMER =================
    private double holdTimer = 0;
    private boolean holdStarted = false;
    private final double HOLD_TIME = 3.0;

    // ================= POSES =================
    private final Pose startPose = new Pose(123.43, 123.03, Math.toRadians(36));
    private final Pose shootPose = new Pose(87.01, 82.29, Math.toRadians(47));
    private final Pose collect1_1 = new Pose(101.4462, 84.0582, Math.toRadians(0));
    private final Pose collect1_2 = new Pose(127.99, 84.39, Math.toRadians(0));
    private final Pose collect2_1 = new Pose(105.4516, 59.6313, Math.toRadians(0));
    private final Pose collect2_2 = new Pose(127.2718, 59.1889, Math.toRadians(0));

    // ================= PATHS =================
    private PathChain driveStartPose;
    private PathChain driveShootPose;

    private PathChain driveCollect1_1;
    private PathChain driveCollect1_2;

    private PathChain driveCollect2_1;
    private PathChain driveCollect2_2;
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

        // returns to shootPose
        driveCollect1_2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1_2, shootPose))
                .setLinearHeadingInterpolation(collect1_2.getHeading(), shootPose.getHeading())
                .build();

        driveCollect2_1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect2_1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect2_1.getHeading())
                .build();

        // returns to shootPose again
        driveCollect2_2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2_2, shootPose))
                .setLinearHeadingInterpolation(collect2_2.getHeading(), shootPose.getHeading())
                .build();
    }

    // ================= STATE SWITCH =================
    public void setPathState(PathState newState) {
        pathState = newState;
        pathStarted = false;

        // reset hold timer whenever leaving states
        holdStarted = false;
    }

    // ================= STATE MACHINE =================
    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOSE:
                if (!follower.isBusy()) {
                    follower.followPath(driveStartPose, true);
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
                    holdTimer = time;

                    // ✅ start ONLY once
                    shooterController.startShooting(3,0,3100);
                }

                // wait until shooter finishes instead of time
                if (!shooterController.isBusy()) {
                    holdStarted = false;
                    if (!shooterController.isBusy()) {
                        holdStarted = false;

                        if (cycleCount == 0) {
                            cycleCount++;
                            setPathState(PathState.DRIVE_COLLECT1_1);
                        } else if (cycleCount == 1) {
                            cycleCount++;
                            setPathState(PathState.DRIVE_COLLECT2_1);
                        } else {
                            cycleCount++;
                            setPathState(PathState.STOP);
                        }
                    }
                }
                break;

            case DRIVE_COLLECT1_1:
                if (!pathStarted) {
                    follower.followPath(driveCollect1_1, false);
                    pathStarted = true;
                }

                intakeMotor.setPower(intakePower);

                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT1_2);
                }
                break;

            case DRIVE_COLLECT1_2:
                if (!pathStarted) {
                    follower.followPath(driveCollect1_2, false);
                    pathStarted = true;
                }

                intakeMotor.setPower(intakePower);

                if (!follower.isBusy()) {
                    setPathState(PathState.HOLD_SHOOTPOSE);
                }
                break;

            case DRIVE_COLLECT2_1:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_1, false);
                    pathStarted = true;
                }

                intakeMotor.setPower(intakePower);

                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT2_2);
                }
                break;

            case DRIVE_COLLECT2_2:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_2, false);
                    pathStarted = true;
                }

                intakeMotor.setPower(intakePower);

                if (!follower.isBusy()) {
                    setPathState(PathState.HOLD_SHOOTPOSE);
                }
                break;
            case STOP:
                // do nothing
                break;
        }
    }

    // ================= INIT =================
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        pathState = PathState.DRIVE_STARTPOSE;

        buildPaths();

        follower.setPose(startPose);
        robot = new org.firstinspires.ftc.teamcode.friends.hardwareMap(hardwareMap);

        helpers = new org.firstinspires.ftc.teamcode.friends.comp.Helpers(robot);

        vision = new org.firstinspires.ftc.teamcode.friends.vision.VisionAlign();

        shooterController = new ShooterController();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}