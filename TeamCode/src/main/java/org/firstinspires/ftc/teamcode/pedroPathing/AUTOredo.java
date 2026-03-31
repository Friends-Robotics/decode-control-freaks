package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Drive Only Auto FIXED")
public class AUTOredo extends OpMode {

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
        DRIVE_COLLECT2_2
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
                }

                // --- ROBOT CAN STILL DO STUFF HERE ---
                intakeMotor.setPower(RintakePower);

                // Example shooter logic (optional)
                // shooterMotor.setPower(0.8);

                if (time - holdTimer >= HOLD_TIME) {
                    holdStarted = false;
                    setPathState(PathState.DRIVE_COLLECT1_1);
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