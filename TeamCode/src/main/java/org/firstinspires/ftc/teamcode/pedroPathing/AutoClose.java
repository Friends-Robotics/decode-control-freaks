package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.friends.components.Intake;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.components.Shooter;
import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.controllers.GoalFusion;
import org.firstinspires.ftc.teamcode.friends.controllers.RobotConstants;
import org.firstinspires.ftc.teamcode.friends.controllers.ShooterController;

@Autonomous
public class AutoClose extends LinearOpMode {

    RobotHardware robotHardware;
    Robot robot;
    Intake intake;
    ShooterController shooterController;
    private Follower follower;

    public static int cycleIndex;
    boolean hasReachedRPM;
    double targetRPM;
    private final ElapsedTime readyTimer = new ElapsedTime();
    double shootTime = 3.25;

    boolean startedPath = false;
    boolean stateJustEntered = true;

    Pose startPose = new Pose(125.000, 118.5, Math.toRadians(36));

    BuildNewCycle cycle;


    enum AutoState{
        START_TO_SHOOT,
        SHOOTING,
        SHOOT_TO_INTAKE,
        INTAKE,
        REVERSE_INTAKE,
        INTAKE_TO_SHOOT,
        PARKING,
        CYCLE
    }

    AutoState currentState = AutoState.START_TO_SHOOT;

    @Override
    public void runOpMode() throws InterruptedException {

        cycleIndex = 0;
        targetRPM = RobotConstants.Shooter.IDLE_RPM;
        hasReachedRPM = false;

        robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);
        shooterController = new ShooterController();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        buildCycle();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            double currentRPM = robot.shooter.getRPM();
            double shooterPower = shooterController.update(targetRPM, currentRPM);
            robot.shooter.setPower(shooterPower);

            robot.shooter.setHoodPosition(RobotConstants.Shooter.CLOSE_HOOD);

            switch(currentState) {

                case START_TO_SHOOT:

                    if (!startedPath) {
                        follower.followPath(cycle.StartShootPath);
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
                        readyTimer.reset();
                        hasReachedRPM = false;
                        stateJustEntered = false;
                    }

                    targetRPM = RobotConstants.Shooter.CLOSE_RPM;

                    if (shooterController.isReady()) {
                        hasReachedRPM = true;
                    }

                    if (hasReachedRPM) {
                        robot.intake.intake();
                        robot.shooter.feed();
                    } else {
                        robot.intake.stop();
                    }

                    if (readyTimer.seconds() > shootTime) {
                        robot.intake.stop();
                        robot.shooter.stopFeed();
                        targetRPM = RobotConstants.Shooter.IDLE_RPM;

                        currentState = AutoState.CYCLE;
                        stateJustEntered = true;
                    }
                    break;

                case SHOOT_TO_INTAKE:

                    if (!startedPath) {
                        follower.followPath(cycle.ShootIntakePath);
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
                        follower.followPath(cycle.IntakePath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        robot.intake.stop();
                        currentState = AutoState.REVERSE_INTAKE;
                        stateJustEntered = true;
                    }
                    break;

                case REVERSE_INTAKE:

                    if (!startedPath) {
                        follower.followPath(cycle.ReverseIntake);
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
                        follower.followPath(cycle.IntakeShootPath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        cycleIndex++; // Updates cycle
                        currentState = AutoState.SHOOTING;
                        stateJustEntered = true;
                    }
                    break;

                case CYCLE:

                    if (cycleIndex < cycle.IntakePoses1.length ) {
                        buildCycle();
                        currentState = AutoState.SHOOT_TO_INTAKE;
                        stateJustEntered = true;
                    } else {
                        currentState = AutoState.PARKING;
                        stateJustEntered = true;
                    }
                    break;

                case PARKING:

                    if (!startedPath) {
                        follower.followPath(cycle.ParkPath);
                        startedPath = true;
                    }

                    targetRPM = 0;
                    robot.shooter.setPower(0);
                    robot.shooter.stopFeed();
                    robot.intake.stop();
                    break;
            }
        }
    }

    // =========================
    //  PATH BUILDER CLASS
    // =========================
    public class BuildNewCycle {


        Pose currentPose;

        Pose[] IntakePoses1 = {
                new Pose(95.500, 84.000 + Tuning.IntakeOffsetY, Math.toRadians(0)),
                new Pose(95.500,60 + Tuning.IntakeOffsetY , Math.toRadians(0)),
                new Pose(95.500,36 + Tuning.IntakeOffsetY , Math.toRadians(0))
        };

        Pose[] IntakePoses2 = {
                new Pose(127.000 + Tuning.IntakeOffsetX, 84.000 + Tuning.IntakeOffsetY, Math.toRadians(0)),
                new Pose(132.500 + Tuning.IntakeOffsetX,60 + Tuning.IntakeOffsetY, Math.toRadians(0)),
                new Pose(132.500 + Tuning.IntakeOffsetX,36 + Tuning.IntakeOffsetY, Math.toRadians(0))
        };

        Pose shootPose = new Pose(102.000, 102.000, Math.toRadians(45));
        Pose parkPose = new Pose(95,12,Math.toRadians(35));

        public PathChain StartShootPath;
        public PathChain ShootIntakePath;
        public PathChain IntakePath;
        public PathChain IntakeShootPath;
        public PathChain ParkPath;

        public PathChain ReverseIntake;
        public BuildNewCycle(Follower follower) {

            if (cycleIndex == 0) {
                currentPose = startPose;
            } else {
                currentPose = follower.getPose();
            }

            StartShootPath = follower.pathBuilder().addPath(
                    new BezierLine(currentPose, shootPose)
            ).setLinearHeadingInterpolation(
                    currentPose.getHeading(),
                    shootPose.getHeading()
            ).build();

            ShootIntakePath = follower.pathBuilder().addPath(
                    new BezierLine(shootPose, IntakePoses1[cycleIndex])
            ).setLinearHeadingInterpolation(
                    shootPose.getHeading(),
                    IntakePoses1[cycleIndex].getHeading()
            ).build();

            IntakePath = follower.pathBuilder().addPath(
                    new BezierLine(IntakePoses1[cycleIndex], IntakePoses2[cycleIndex])
            ).setLinearHeadingInterpolation(
                    IntakePoses1[cycleIndex].getHeading(),
                    IntakePoses2[cycleIndex].getHeading()
            ).build();

            IntakeShootPath = follower.pathBuilder().addPath(
                    new BezierLine(IntakePoses2[cycleIndex], shootPose)
            ).setLinearHeadingInterpolation(
                    IntakePoses2[cycleIndex].getHeading(),
                    shootPose.getHeading()
            ).build();

            ReverseIntake = follower.pathBuilder().addPath(
                    new BezierLine(IntakePoses2[cycleIndex], IntakePoses1[cycleIndex])
            ).setLinearHeadingInterpolation(
                    IntakePoses2[cycleIndex].getHeading(),
                    IntakePoses1[cycleIndex].getHeading()
            ).build();

            ParkPath = follower.pathBuilder().addPath(
                    new BezierLine(IntakePoses2[cycleIndex], parkPose)
            ).setLinearHeadingInterpolation(
                    IntakePoses2[cycleIndex].getHeading(),
                    parkPose.getHeading()
            ).build();
        }
    }

    public void buildCycle() {
        cycle = new BuildNewCycle(follower);
    }

    @Config
    public static class Tuning{
        public static double IntakeOffsetY = 4.5;
        public static double IntakeOffsetX = 8; //Change to 7
    }
}