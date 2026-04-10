package org.firstinspires.ftc.teamcode.pedroPathing;

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
    double shootTime = 5.0; // Tune
    boolean startedPath = false;
    boolean stateJustEntered = false;

    enum AutoState{
        START_TO_SHOOT,
        SHOOTING,
        SHOOT_TO_INTAKE,
        INTAKE,
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

        robot.pinpointDriver.update();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildCycle();

        waitForStart();

        while (opModeIsActive())
        {
            follower.update();

            double currentRPM = robot.shooter.getRPM();
            double shooterPower = shooterController.update(targetRPM, currentRPM);
            robot.shooter.setPower(shooterPower);


            switch(currentState)
            {
                case START_TO_SHOOT:

                    if (!startedPath) {
                        follower.followPath(StartShootPath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        currentState = AutoState.SHOOTING;
                        readyTimer.reset();
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

                        currentState = AutoState.SHOOT_TO_INTAKE;
                        stateJustEntered = true;
                    }

                    break;

                case SHOOT_TO_INTAKE:

                    if (!startedPath) {
                        follower.followPath(ShootIntakePath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        robot.intake.intake();
                        currentState = AutoState.INTAKE;
                    }
                    break;

                case INTAKE:

                    if (!startedPath) {
                        follower.followPath(IntakePath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        robot.intake.stop();
                        currentState = AutoState.INTAKE_TO_SHOOT;
                    }
                    break;

                case INTAKE_TO_SHOOT:

                    if (!startedPath) {
                        follower.followPath(IntakeShootPath);
                        startedPath = true;
                    }

                    if (!follower.isBusy()) {
                        startedPath = false;
                        cycleIndex++;
                        currentState = AutoState.CYCLE;
                    }
                    break;

                case CYCLE:

                    if (cycleIndex < IntakePoses1.length && cycleIndex < IntakePoses2.length) {
                        buildCycle();
                        readyTimer.reset();
                        hasReachedRPM = false;
                        currentState = AutoState.SHOOTING;
                    } else {
                        currentState = AutoState.PARKING;
                    }
                    break;

                case PARKING:

                    if (!startedPath) {
                        follower.followPath(ParkPath);
                        startedPath = true;
                    }

                    robot.shooter.setPower(0);
                    robot.shooter.stopFeed();
                    robot.intake.stop();
                    break;
            }
        }
    }

    public PathChain StartShootPath;
    public PathChain ShootIntakePath;
    public PathChain IntakePath;
    public PathChain IntakeShootPath;
    public PathChain ParkPath;
    Pose[] IntakePoses1 = {
            new Pose(100.000,84.000),
    };
    Pose[] IntakePoses2 = {
            new Pose(127.000,84.000)
    };
    Pose startPose = new Pose(122.500,122.500,37);
    Pose shootPose = new Pose(108.000,107.500,45);
    Pose parkPose = new Pose(96,120,90);

    public class BuildNewCycle {
        Pose currentPose = follower.getPose();
        public BuildNewCycle(Follower follower) {
            StartShootPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(currentPose.getX(), currentPose.getY()),

                                    new Pose(shootPose.getX(), shootPose.getY())
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(currentPose.getHeading()), Math.toRadians(shootPose.getHeading()))

                    .build();

            ShootIntakePath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(shootPose.getX(), shootPose.getY()),

                                    new Pose(IntakePoses1[cycleIndex].getX(),IntakePoses1[cycleIndex].getY())
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(shootPose.getHeading()), Math.toRadians(IntakePoses1[cycleIndex].getHeading()))

                    .build();

            IntakePath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(IntakePoses1[cycleIndex].getX(),IntakePoses1[cycleIndex].getY()),

                                    new Pose(IntakePoses2[cycleIndex].getX(),IntakePoses2[cycleIndex].getY())
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(IntakePoses2[cycleIndex].getHeading()), Math.toRadians(IntakePoses2[cycleIndex].getHeading()))

                    .build();

            IntakeShootPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(IntakePoses2[cycleIndex].getX(),IntakePoses2[cycleIndex].getY()),

                                    new Pose(shootPose.getX(), shootPose.getY())
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(IntakePoses2[cycleIndex].getHeading()), Math.toRadians(shootPose.getHeading()))

                    .build();

            ParkPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(IntakePoses2[cycleIndex].getX(), IntakePoses2[cycleIndex].getY()),

                                    new Pose(parkPose.getX(), parkPose.getY())
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(IntakePoses2[cycleIndex].getHeading()), Math.toRadians(parkPose.getHeading()))

                    .build();


        }
    }
    public void buildCycle() {
        new BuildNewCycle(follower);
    }

}
