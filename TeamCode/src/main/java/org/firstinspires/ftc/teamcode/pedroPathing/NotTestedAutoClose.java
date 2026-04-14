package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
public class NotTestedAutoClose extends LinearOpMode {

    RobotHardware robotHardware;
    Robot robot;
    Intake intake;
    ShooterController shooterController;
    private Follower follower;

    public static int cycleIndex;
    public static int maxCycles;
    boolean hasReachedRPM;
    double targetRPM;
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    double shootTime = 3.25;
    double gateWaitTime = 1.5;

    boolean startedPath = false;
    boolean stateJustEntered = true;

    Pose startPose = p(125.000, 118.5, 36);
    public static boolean isBlue;

    BuildNewCycle cycle;

    enum AutoState{
        START_TO_SHOOT,
        SHOOTING,
        SHOOT_TO_INTAKE,
        INTAKE,
        INTAKE_TO_SHOOT,
        SHOOT_TO_GATE,
        GATE_TO_SHOOT,
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

        PickMode();

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

                case SHOOT_TO_GATE:

                    if(!startedPath) {
                        follower.followPath(cycle.ShootGatePath);
                        startedPath = true;
                    }

                    if(!follower.isBusy()){
                        startedPath = false;
                        robot.intake.intake();
                        currentState = AutoState.GATE_TO_SHOOT;
                        stateJustEntered = true;
                    }
                    break;

                case GATE_TO_SHOOT:

                    if (stateJustEntered) {
                        gateTimer.reset();
                        stateJustEntered = false;
                    }

                    if (gateTimer.seconds() < gateWaitTime) {
                        robot.intake.intake();
                    } else {
                        robot.intake.stop();

                        if(!startedPath){
                            follower.followPath(cycle.GateShootPath);
                            startedPath = true;
                        }
                    }

                    if(!follower.isBusy())
                    {
                        startedPath = false;
                        cycleIndex++;
                        currentState = AutoState.SHOOTING;
                        stateJustEntered = true;
                    }
                    break;

                case CYCLE:

                    if (cycleIndex < maxCycles ) {
                        if(maxCycles == 3 && cycleIndex == 2) //Checks for whether we are doing 12 ball and we are on our 3rd intake
                        {
                            buildCycle();
                            currentState = AutoState.SHOOT_TO_GATE;
                            stateJustEntered = true;
                        }
                        else{
                            buildCycle();
                            currentState = AutoState.SHOOT_TO_INTAKE;
                            stateJustEntered = true;
                        }

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
                p(95.500, 84.000 + Tuning.IntakeOffsetY, 0),
                p(95.500,60 + Tuning.IntakeOffsetY , 0),
                //p(95.500,36 + Tuning.IntakeOffsetY , 0) don't need for updated 12 ball
        };

        Pose[] IntakePoses2 = {
                p(127.000 + Tuning.IntakeOffsetX, 84.000 + Tuning.IntakeOffsetY, 0),
                p(132.500 + Tuning.IntakeOffsetX,60 + Tuning.IntakeOffsetY, 0),
                //p(132.500 + Tuning.IntakeOffsetX,36 + Tuning.IntakeOffsetY, 0)
        };

        Pose shootPose = p(102.000, 102.000, 45);
        Pose parkPose = p(95,12,35);
        Pose gatePose = p(135,60,60);

        public PathChain StartShootPath;
        public PathChain ShootIntakePath;
        public PathChain IntakePath;
        public PathChain IntakeShootPath;
        public PathChain ShootGatePath;
        public PathChain GateShootPath;

        public PathChain ParkPath;

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

            ShootGatePath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(shootPose.getX(),shootPose.getY()),
                                    new Pose(gatePose.getX(), gatePose.getY())
                            )
                    ).setLinearHeadingInterpolation(shootPose.getHeading() , gatePose.getHeading())
                    .build();

            GateShootPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(gatePose.getX(), gatePose.getY()),

                                    new Pose(shootPose.getX(), shootPose.getY())
                            )
                    ).setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())

                    .build();

            ParkPath = follower.pathBuilder().addPath(
                    new BezierLine(IntakePoses2[cycleIndex], parkPose)
            ).setLinearHeadingInterpolation(
                    IntakePoses2[cycleIndex].getHeading(),
                    parkPose.getHeading()
            ).build();
        }
    }

    //Matthew please don't CrashOut
    public void buildCycle() {
        cycle = new BuildNewCycle(follower);
    }

    //Calculates mirrored pose
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

    public void PickMode(){
        String[] alliances = {"RED", "BLUE"};
        int allianceIndex = 0;

        int[] ballOptions = {3, 6, 9, 12};
        int ballIndex = 2; // default = 9 ball since that one works

        int menuIndex = 0; // 0 = alliance, 1 = balls

        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastLeft = false;
        boolean lastRight = false;

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.dpad_up && !lastUp) {
                menuIndex--;
            }
            if (gamepad1.dpad_down && !lastDown) {
                menuIndex++;
            }

            if (gamepad1.dpad_left && !lastLeft) {
                if (menuIndex == 0) allianceIndex--;
                if (menuIndex == 1) ballIndex--;
            }

            if (gamepad1.dpad_right && !lastRight) {
                if (menuIndex == 0) allianceIndex++;
                if (menuIndex == 1) ballIndex++;
            }


            if (menuIndex < 0) menuIndex = 1;
            if (menuIndex > 1) menuIndex = 0;

            if (allianceIndex < 0) allianceIndex = alliances.length - 1;
            if (allianceIndex >= alliances.length) allianceIndex = 0;

            if (ballIndex < 0) ballIndex = ballOptions.length - 1;
            if (ballIndex >= ballOptions.length) ballIndex = 0;

            // Display menu
            telemetry.addLine("=== AUTO SELECT ===");

            telemetry.addData(
                    menuIndex == 0 ? "> Alliance" : "  Alliance",
                    alliances[allianceIndex]
            );

            telemetry.addData(
                    menuIndex == 1 ? "> Balls" : "  Balls",
                    ballOptions[ballIndex]
            );

            telemetry.addLine("Use D-pad to change");
            telemetry.update();

            // Save button states
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
        }

        isBlue = alliances[allianceIndex].equals("BLUE");
        int selectedBalls = ballOptions[ballIndex];
        maxCycles = (selectedBalls/3) -1; // At 3 ball there is no cycles
    }


    @Config
    public static class Tuning{
        public static double IntakeOffsetY = 5.5;
        public static double IntakeOffsetX = 8; //Change to 7
    }
}
