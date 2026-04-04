package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "SAFE Auto 1 (PINPOINT FIXED)")
public class FullAuto extends OpMode {

    private Follower follower;
    private boolean pathStarted = false;

    private DcMotor FLM, BLM, FRM, BRM;
    private DcMotor intakeMotor;

    public enum PathState {
        DRIVE_STARTPOS,
        DRIVE_SHOOTPOS,
        DRIVE_COLLECT1_2,
        DRIVE_SHOOT2,
        DRIVE_COLLECT2_1,
        DRIVE_COLLECT2_2
    }

    PathState pathState;

    private final Pose startPose = new Pose(123.42994011976043, 123.02754491017964, Math.toRadians(36));
    private final Pose shootPose = new Pose(87.00958083832337, 82.29101796407181, Math.toRadians(0));
    private final Pose collect1_1 = new Pose(127.99640718562875, 84.38682634730539, Math.toRadians(47));
    private final Pose collect1_2 = new Pose(87.00958083832337, 82.29101796407181, Math.toRadians(47));
    private final Pose shoot2 = new Pose(102.60000000000002, 62.90898203592813, Math.toRadians(0));
    private final Pose collect2_1 = new Pose(136.73652694610774, 62.408383233532966, Math.toRadians(0));
    private final Pose collect2_2 = new Pose(87.00958083832337, 82.29101796407181, Math.toRadians(47));

    private PathChain driveStartPosShootPos, driveCollect1_1, driveCollect1_2,
            driveShoot2, driveCollect2_1, driveCollect2_2;

    public void buildPaths() {

        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
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
    }

    public void stopDriveMotors() {
        FLM.setPower(0);
        BLM.setPower(0);
        FRM.setPower(0);
        BRM.setPower(0);
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
                stopDriveMotors();

                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT1_2);
                }
                break;

            case DRIVE_COLLECT1_2:
                if (!pathStarted) {
                    follower.followPath(driveShoot2, false);
                    pathStarted = true;
                }
                stopDriveMotors();

                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_SHOOT2);
                }
                break;

            case DRIVE_SHOOT2:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_1, false);
                    pathStarted = true;
                }
                stopDriveMotors();

                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT2_1);
                }
                break;

            case DRIVE_COLLECT2_1:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_2, false);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT2_2);
                }
                break;

            case DRIVE_COLLECT2_2:
                stopDriveMotors();
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathStarted = false;
    }

    @Override
    public void init() {

        pathState = PathState.DRIVE_STARTPOS;

        // ✅ THIS IS THE FIX:
           // NO hardwareMap, NO Pinpoint

        FLM = hardwareMap.get(DcMotor.class, "FLM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        BRM = hardwareMap.get(DcMotor.class, "BRM");

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        pathStarted = false;
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