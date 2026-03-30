package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "12 Ball Auto")
public class HBAUTO extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private boolean pathStarted = false;
    private DcMotor intakeMotor;

    public enum PathState {
        DRIVE_STARTPOS,
        DRIVE_SHOOTPOS,
        WAIT_SHOOTPOS,
        DRIVE_COLLECT1_1,
        DRIVE_COLLECT1_2,
        DRIVE_SHOOT2,
        WAIT_SHOOT2,
        DRIVE_COLLECT2_1,
        DRIVE_COLLECT2_2,
        WAIT_AFTER_COLLECT2_2,
        DRIVE_SHOOT3_1,
        DRIVE_SHOOT3_2,
        WAIT_SHOOT3_2,
        DRIVE_COLLECT3,
        DRIVE_SHOOT4,
        WAIT_SHOOT4,
        DRIVE_ENDPOS
    }

    PathState pathState;
    private final float intakePower = 0.8f;

    private final Pose startPose = new Pose(123.42994011976043, 123.02754491017964, Math.toRadians(44));
    private final Pose shootPose = new Pose(92.70059880239522, 85.91257485029936, Math.toRadians(47));
    private final Pose collect1_1 = new Pose(104.82634730538922, 59.75808383233533, Math.toRadians(0));
    private final Pose collect1_2 = new Pose(125.27305389221557, 59.159281437125756, Math.toRadians(0));
    private final Pose shoot2 = new Pose(88.7544910179641, 82.36646706586826, Math.toRadians(47));
    private final Pose collect2_1 = new Pose(122.62634730538922, 63.75329341317365, Math.toRadians(35));
    private final Pose collect2_2 = new Pose(133.8251497005988, 60.44071856287424, Math.toRadians(31));
    private final Pose shoot3_1 = new Pose(122.62634730538922, 63.75329341317365, Math.toRadians(35));
    private final Pose shoot3_2 = new Pose(88.7544910179641, 82.36646706586826, Math.toRadians(47));
    private final Pose collect3 = new Pose(122.82488479262672, 83.45622119815667, Math.toRadians(0));
    private final Pose shoot4 = new Pose(88.7544910179641, 82.36646706586826, Math.toRadians(47));
    private final Pose endPos = new Pose(115.73496509285575, 114.49035569414167, Math.toRadians(90));

    private PathChain driveStartPosShootPos, driveCollect1_1, driveCollect1_2, driveShoot2,
            driveCollect2_1, driveCollect2_2, driveshoot3_1, driveshoot3_2,
            drivecollect3, driveshoot4, driveendpos;

    private double getShortestHeading(double start, double end) {
        double delta = end - start;
        if (delta > Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;
        return start + delta;
    }

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),
                        getShortestHeading(startPose.getHeading(), shootPose.getHeading()))
                .build();

        driveCollect1_1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1_1))
                .build();

        driveCollect1_2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1_1, collect1_2))
                .build();

        driveShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1_2, shoot2))
                .build();

        driveCollect2_1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, collect2_1))
                .build();

        driveCollect2_2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2_1, collect2_2))
                .build();

        driveshoot3_1 = follower.pathBuilder()
                .addPath(new BezierLine(collect2_2, shoot3_1))
                .build();

        driveshoot3_2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3_1, shoot3_2))
                .build();

        drivecollect3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3_2, collect3))
                .build();

        driveshoot4 = follower.pathBuilder()
                .addPath(new BezierLine(collect3, shoot4))
                .build();

        driveendpos = follower.pathBuilder()
                .addPath(new BezierLine(shoot4, endPos))
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {

            case DRIVE_STARTPOS:
                if (!pathStarted) {
                    follower.followPath(driveStartPosShootPos, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_SHOOTPOS);
                }
                break;

            case DRIVE_SHOOTPOS:
                if (!pathStarted) {
                    follower.followPath(driveCollect1_1, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_SHOOTPOS);
                }
                break;

            case WAIT_SHOOTPOS:
                if (pathTimer.getElapsedTimeSeconds() >= 0.8) {
                    setPathState(PathState.DRIVE_COLLECT1_1);
                }
                break;

            case DRIVE_COLLECT1_1:
                if (!pathStarted) {
                    follower.followPath(driveCollect1_2, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_SHOOT2);
                }
                break;

            case DRIVE_SHOOT2:
                if (!pathStarted) {
                    follower.followPath(driveShoot2, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_SHOOT2);
                }
                break;

            case WAIT_SHOOT2:
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(PathState.DRIVE_COLLECT2_1);
                }
                break;

            case DRIVE_COLLECT2_1:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_1, false);
                    pathStarted = true;
                }
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
                    setPathState(PathState.WAIT_AFTER_COLLECT2_2);
                }
                break;

            case WAIT_AFTER_COLLECT2_2:
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    intakeMotor.setPower(0);
                    setPathState(PathState.DRIVE_SHOOT3_1);
                }
                break;

            case DRIVE_SHOOT3_1:
                if (!pathStarted) {
                    follower.followPath(driveshoot3_1, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_SHOOT3_2);
                }
                break;

            case DRIVE_SHOOT3_2:
                if (!pathStarted) {
                    follower.followPath(driveshoot3_2, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_SHOOT3_2);
                }
                break;

            case WAIT_SHOOT3_2:
                if (pathTimer.getElapsedTimeSeconds() >= 0.8) {
                    setPathState(PathState.DRIVE_COLLECT3);
                }
                break;

            case DRIVE_COLLECT3:
                if (!pathStarted) {
                    follower.followPath(drivecollect3, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_SHOOT4);
                }
                break;

            case DRIVE_SHOOT4:
                if (!pathStarted) {
                    follower.followPath(driveshoot4, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_SHOOT4);
                }
                break;

            case WAIT_SHOOT4:
                if (pathTimer.getElapsedTimeSeconds() >= 0.8) {
                    setPathState(PathState.DRIVE_ENDPOS);
                }
                break;

            case DRIVE_ENDPOS:
                if (!pathStarted) {
                    follower.followPath(driveendpos, false);
                    pathStarted = true;
                }
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        pathStarted = false;
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}