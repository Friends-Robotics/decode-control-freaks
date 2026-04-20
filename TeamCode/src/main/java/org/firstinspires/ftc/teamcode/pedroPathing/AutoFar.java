package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.friends.components.Intake;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.controllers.ShooterController;

@Autonomous
public class AutoFar extends LinearOpMode{
    RobotHardware robotHardware;
    Robot robot;
    Intake intake;
    ShooterController shooterController;
    private Follower follower;

    boolean hasReachedRPM;
    double targetRPM;
    private final ElapsedTime readyTimer = new ElapsedTime();
    double shootTime = 7; // try to lower

    ElapsedTime shotTimer = new ElapsedTime();
    boolean isFeeding = false;
    double shotDelay = 0.3; //between shots TUNE
    int timesEnteredShooting;

    boolean startedPath = false;
    boolean stateJustEntered = true;

    static Pose startPose = new Pose(82, 9, Math.toRadians(90));

    public static boolean isBlue;

    Paths path;

    enum AutoState{
        START_TO_SHOOT,
        SHOOTING,
        SHOOT_TO_PREP,
        INTAKE,
        INTAKE_TO_SHOOT,
        PARKING_TO_GATE
    }

    AutoState currentState = AutoState.START_TO_SHOOT;

    @Override
    public void runOpMode() throws InterruptedException {

        timesEnteredShooting = 0;
        targetRPM = RobotConstants.Shooter.IDLE_RPM;
        hasReachedRPM = false;

        robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);
        shooterController = new ShooterController();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        BuildPaths();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            double currentRPM = robot.shooter.getRPM();
            double shooterPower = shooterController.update(targetRPM, currentRPM);
            robot.shooter.setPower(shooterPower);

            robot.shooter.setHoodPosition(RobotConstants.Shooter.FAR_HOOD);

            switch(currentState) {

                case START_TO_SHOOT:

                    if (!startedPath) {
                        follower.followPath(path.StartShootPath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        currentState = AutoState.SHOOTING;
                        stateJustEntered = true;
                    }
                    break;

                case SHOOTING:

                    if (stateJustEntered) {
                        timesEnteredShooting++;
                        readyTimer.reset();
                        hasReachedRPM = false;
                        stateJustEntered = false;
                    }

                    targetRPM = RobotConstants.Shooter.FAR_RPM;

                    if (shooterController.isReady()) {
                        hasReachedRPM = true;
                    }

                    if (hasReachedRPM) {
                        if (!isFeeding) {
                            // Start feeding a shot
                            robot.intake.intake();
                            robot.shooter.feed();
                            shotTimer.reset();
                            isFeeding = true;
                        }

                        // Stop feeding after a short burst
                        if (isFeeding && shotTimer.seconds() > shotDelay) { // feed duration
                            robot.intake.stop();
                            robot.shooter.stopFeed();
                            isFeeding = false;

                            // Force RPM to be re-validated before next shot
                            hasReachedRPM = false;
                        }
                    }

                    if (readyTimer.seconds() > shootTime) {
                        robot.intake.stop();
                        robot.shooter.stopFeed();
                        targetRPM = RobotConstants.Shooter.IDLE_RPM;
                        if(timesEnteredShooting == 1)
                        {
                            currentState = AutoState.SHOOT_TO_PREP;
                        }
                        else{
                            currentState = AutoState.PARKING_TO_GATE;
                        }
                        stateJustEntered = true;
                    }
                    break;

                case SHOOT_TO_PREP:

                    if (!startedPath) {
                        follower.followPath(path.ShootPrepPath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        robot.intake.intake();
                        currentState = AutoState.INTAKE;
                        stateJustEntered = true;
                    }
                    break;

                case INTAKE:

                    if (!startedPath) {
                        follower.followPath(path.PrepIntakePath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        robot.intake.stop();
                        currentState = AutoState.INTAKE_TO_SHOOT;
                        stateJustEntered = true;
                    }
                    break;

                case INTAKE_TO_SHOOT:

                    if (!startedPath) {
                        follower.followPath(path.IntakeShootPath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        currentState = AutoState.SHOOTING;
                        stateJustEntered = true;
                    }
                    break;

                case PARKING_TO_GATE:
                    if (!startedPath) {
                        follower.followPath(path.ShootParkPath);
                        startedPath = true;
                    }

                    targetRPM = 0;
                    robot.shooter.setPower(0);
                    robot.shooter.stopFeed();
                    robot.intake.stop();
                    break;
            }
            telemetry.addData("Current Pose", follower.getPose());
            telemetry.addData("Current path",follower.getCurrentPath());
            telemetry.addData("Current Drawn", robot.getCurrent(CurrentUnit.AMPS) );
            telemetry.addData("CurrentRPM", shooterController.getFilteredRPM());
            telemetry.addData("TargetRPM", shooterController.getTargetRPM());
            telemetry.update();
        }
    }

    public static class Paths {
        Pose shootPose = new Pose(82,25, Math.toRadians(70));
        Pose prepPose = new Pose(82,35 + Tuning.IntakeOffsetY,Math.toRadians(0));
        Pose endIntakePose = new Pose(125 + Tuning.IntakeOffsetX,35 + Tuning.IntakeOffsetY, Math.toRadians(0));
        Pose parkPose = new Pose(130,40, Math.toRadians(90));

        public PathChain StartShootPath;
        public PathChain ShootPrepPath;
        public PathChain PrepIntakePath;
        public PathChain IntakeShootPath;
        public PathChain ShootParkPath;

        public Paths(Follower follower) {
            StartShootPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(startPose.getX(), startPose.getY()),

                                    new Pose(shootPose.getX(), shootPose.getY())
                            )
                    ).setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())

                    .build();

            ShootPrepPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(shootPose.getX(), shootPose.getY()),

                                    new Pose(prepPose.getX(), prepPose.getY())
                            )
                    ).setLinearHeadingInterpolation(shootPose.getHeading(), prepPose.getHeading())

                    .build();

            PrepIntakePath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(prepPose.getX(), prepPose.getY()),

                                    new Pose(endIntakePose.getX(), endIntakePose.getY())
                            )
                    ).setLinearHeadingInterpolation(prepPose.getHeading(), endIntakePose.getHeading())

                    .build();

            IntakeShootPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(endIntakePose.getX(), endIntakePose.getY()),

                                    new Pose(shootPose.getX(), shootPose.getY())
                            )
                    ).setLinearHeadingInterpolation(endIntakePose.getHeading(), shootPose.getHeading())

                    .build();

            ShootParkPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(shootPose.getX(), shootPose.getY()),

                                    new Pose(parkPose.getX(), parkPose.getY())
                            )
                    ).setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())

                    .build();
        }
    }
    public void BuildPaths() {
        path = new Paths(follower);
    }

    @Config
    public static class Tuning{
        public static double IntakeOffsetY = 22.5;
        public static double IntakeOffsetX = 5;
    }

    public Pose mirrorPose(Pose p) {
        return new Pose(
                144.0 - p.getX(), //144 is full length of arena
                p.getY(),
                Math.PI - p.getHeading() //PI is Pi
        );
    }
    public Pose p(double x, double y, double headingDeg) {
        Pose pose = new Pose(x, y, Math.toRadians(headingDeg));
        return isBlue ? mirrorPose(pose) : pose;
    }
}
