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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SAFE Auto")
public class AUTOredo extends OpMode {

    private Follower follower;
    private boolean pathStarted = false;

    // Drive motors
    private DcMotor FLM, BLM, FRM, BRM;

    private DcMotor intakeMotor;
    private DcMotorEx shooterMotor1, shooterMotor2;
    private Servo rampServo;

    // Ramp positions
    private final double rampRestPosition = 0.0;
    private final double rampShootPosition = 0.7;

    // Shooter/feeding logic
    private boolean revPhase = false;
    private boolean feeding = false;
    private double revStartTime = 0;
    private final double shooterTargetRPM = 3200; // target RPM
    private final double intakePower = 0.8f;

    public enum PathState {
        DRIVE_STARTPOS,
        DRIVE_SHOOTPOS,
        DRIVE_COLLECT1_2,
        DRIVE_SHOOT2,
        DRIVE_COLLECT2_1,
        DRIVE_COLLECT2_2
    }

    PathState pathState;

    // Original poses
    private final Pose startPose = new Pose(123.42994011976043, 123.02754491017964, Math.toRadians(36));
    private final Pose shootPose = new Pose(87.00958083832337, 82.29101796407181, Math.toRadians(0));
    private final Pose collect1_1 = new Pose(127.99640718562875, 84.38682634730539, Math.toRadians(47));
    private final Pose collect1_2 = new Pose(87.00958083832337, 82.29101796407181, Math.toRadians(47));
    private final Pose shoot2 = new Pose(102.60000000000002, 62.90898203592813 , Math.toRadians(0));
    private final Pose collect2_1 = new Pose(136.73652694610774 , 62.408383233532966, Math.toRadians(0));
    private final Pose collect2_2 = new Pose(87.00958083832337, 82.29101796407181, Math.toRadians(47));

    private PathChain driveStartPosShootPos, driveCollect1_1, driveCollect1_2, driveShoot2,
            driveCollect2_1, driveCollect2_2;

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
                .setLinearHeadingInterpolation(shootPose.getHeading(),
                        getShortestHeading(shootPose.getHeading(), collect1_1.getHeading()))
                .build();

        driveCollect1_2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1_1, collect1_2))
                .setLinearHeadingInterpolation(collect1_1.getHeading(),
                        getShortestHeading(collect1_1.getHeading(), collect1_2.getHeading()))
                .build();

        driveShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1_2, shoot2))
                .setLinearHeadingInterpolation(collect1_2.getHeading(),
                        getShortestHeading(collect1_2.getHeading(), shoot2.getHeading()))
                .build();

        driveCollect2_1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, collect2_1))
                .setLinearHeadingInterpolation(shoot2.getHeading(),
                        getShortestHeading(shoot2.getHeading(), collect2_1.getHeading()))
                .build();

        driveCollect2_2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2_1, collect2_2))
                .setLinearHeadingInterpolation(collect2_1.getHeading(),
                        getShortestHeading(collect2_1.getHeading(), collect2_2.getHeading()))
                .build();
    }

    private void handleShooterRampFeeding(Pose targetPose) {
        final double kP = 0.0005; // simple P-controller for RPM

        // Current velocities in RPM
        double rpm1 = shooterMotor1.getVelocity() * 60.0;
        double rpm2 = shooterMotor2.getVelocity() * 60.0;

        // P-controller to hold 3200 RPM
        double power1 = -1.0 + kP * (shooterTargetRPM - Math.abs(rpm1));
        double power2 = -1.0 + kP * (shooterTargetRPM - Math.abs(rpm2));
        power1 = Math.max(-1.0, Math.min(1.0, power1));
        power2 = Math.max(-1.0, Math.min(1.0, power2));

        shooterMotor1.setPower(power1);
        shooterMotor2.setPower(power2);

        // Compute distance to target pose
        double distance = Math.hypot(follower.getPose().getX() - targetPose.getX(),
                follower.getPose().getY() - targetPose.getY());

        // Start feeding when RPM reached and close to target
        if (!feeding && Math.abs(rpm1) >= shooterTargetRPM && Math.abs(rpm2) >= shooterTargetRPM && distance < 1.0) {
            rampServo.setPosition(rampShootPosition);
            intakeMotor.setPower(intakePower);
            revStartTime = getRuntime();
            feeding = true;
        }

        // Feed for 5 seconds
        if (feeding) {
            if (getRuntime() - revStartTime > 5.0) {
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
                rampServo.setPosition(rampRestPosition);
                intakeMotor.setPower(0);
                revPhase = false;
                feeding = false;
            }
        }
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
                handleShooterRampFeeding(collect1_2);
                if(!follower.isBusy() && !feeding) {
                    setPathState(PathState.DRIVE_COLLECT1_2);
                }
                break;

            case DRIVE_COLLECT1_2:
                // Pause to shoot
                stopDriveMotors();
                handleShooterRampFeeding(collect1_2);
                if(!feeding) {
                    if (!pathStarted) {
                        follower.followPath(driveShoot2, false);
                        pathStarted = true;
                    }
                    if(!follower.isBusy()) {
                        setPathState(PathState.DRIVE_SHOOT2);
                    }
                }
                break;

            case DRIVE_SHOOT2:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_1, false);
                    pathStarted = true;
                }
                stopDriveMotors(); // stop for second shooting
                handleShooterRampFeeding(shoot2);
                if(!follower.isBusy() && !feeding) {
                    setPathState(PathState.DRIVE_COLLECT2_1);
                }
                break;

            case DRIVE_COLLECT2_1:
                if (!pathStarted) {
                    follower.followPath(driveCollect2_2, false);
                    pathStarted = true;
                }
                if(!follower.isBusy()) {
                    setPathState(PathState.DRIVE_COLLECT2_2);
                }
                break;

            case DRIVE_COLLECT2_2:
                // Pause to shoot
                stopDriveMotors();
                handleShooterRampFeeding(collect2_2);
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathStarted = false;
    }

    @Override
    public void init() {
        // Set initial path state
        pathState = PathState.DRIVE_STARTPOS;

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);

        // Initialize drive motors
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        BRM = hardwareMap.get(DcMotor.class, "BRM");

        // Initialize other hardware
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        rampServo = hardwareMap.get(Servo.class, "Feeder");

        // Build all paths
        buildPaths();

        // Set follower starting pose
        follower.setPose(startPose);

        // Debug telemetry
        telemetry.addData("Init", "Follower initialized");
        telemetry.addData("Start X", startPose.getX());
        telemetry.addData("Start Y", startPose.getY());
        telemetry.addData("Start Heading", Math.toDegrees(startPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        pathStarted = false;
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // Debug telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Started?", pathStarted);
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        double distanceToShoot = Math.hypot(
                follower.getPose().getX() - shootPose.getX(),
                follower.getPose().getY() - shootPose.getY());
        telemetry.addData("Distance to Shoot Pose", distanceToShoot);
        telemetry.update();
    }
}