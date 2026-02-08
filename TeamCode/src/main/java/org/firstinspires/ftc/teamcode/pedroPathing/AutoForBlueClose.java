package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;

@Autonomous(name = "Pedro Multi-Ball")
public class AutoForBlueClose extends LinearOpMode { //FOR BLUE ALLIANCE CLOSE

    hardwareMap robot; // Uses the Hardware map in teleOp
    Follower follower; // a Pedropathing thing that allows the robot to "follow" the paths
    Limelight3A limelight;
    VisionAlign visionAlign;


    enum AutoState { //Fun stuff

        VISION_ALIGN,
        PRELOAD_DRIVE_TO_SHOOT,
        PRELOAD_SPIN_UP,
        PRELOAD_FEED,
        DRIVE_TO_INTAKE,
        DRIVE_TO_SHOOT,
        SPIN_UP_SHOOTER,
        FEED_BALL,
        DONE
    }

    AutoState currentState = AutoState.PRELOAD_DRIVE_TO_SHOOT; // Makes sure the first state is the first state

    ElapsedTime stateTimer = new ElapsedTime(); // Controls the feeder duration without using sleep()

    static final double TARGET_RPM = 2000;
    static final double RPM_TOLERANCE = 100;
    static final double FEED_TIME = 0.4;

    int VisionCount = 0;
    static final int MAX_CYCLES = 3; // Does runs through the state machine 3 times
    int cycleIndex = 0; // This checks through the intakeposes array
    int ballsShot = 0;

    Pose startPose = new Pose(16, 130, 135);
    Pose shootPose = new Pose(60, 84, Math.toRadians(135)); // At this position the goal is 70 inches away

    Pose[] intakePoses = { // Cycle index cycles through these poses
            new Pose(35, 85, 180),
            new Pose(35, 60, 180),
            new Pose(35, 35, 180)
    };
    Pose[] intakePoses2 = { // Cycle index cycles through these poses
            new Pose(8, 85, 180),
            new Pose(8, 60, 180),
            new Pose(8, 35, 180)
    };

    PathChain intakePath; // Paths are built on the fly and change every cycle
    PathChain intakePath2;
    PathChain shootPath;
    PathChain StartShootPath;



    @Override
    public void runOpMode() {

        robot = new hardwareMap(hardwareMap); // Starts robot hardware
        follower = Constants.createFollower(hardwareMap); // Creates pedro follower
        follower.setPose(startPose);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart(); // PRESS START queue geometry dash

        robot.prepfeedBall();
        robot.setShooterRPM(TARGET_RPM); // Allows the shooter to be sped up before auto
        stateTimer.reset();


        while (opModeIsActive()) {
            follower.update(); // Updates follower position and must be called every loop

            switch (currentState) {

                /* ---------- PRELOAD ---------- */

                case VISION_ALIGN:
                    LLResult result = limelight.getLatestResult();

                    // Update visionAlign
                    visionAlign.update(result, true);

                    robot.turretMotor.setPower(visionAlign.turretPower);

                    /*------Mecanum Drive------*/

                    double drive  = visionAlign.drivePower;
                    double strafe = visionAlign.strafePower;
                    double rotate = 0; // lock rotation while aligning

                    double fl = drive + strafe + rotate;
                    double fr = drive - strafe - rotate;
                    double bl = drive - strafe + rotate;
                    double br = drive + strafe - rotate;

                    fl = Range.clip(fl, -1, 1);
                    fr = Range.clip(fr, -1, 1);
                    bl = Range.clip(bl, -1, 1);
                    br = Range.clip(br, -1, 1);

                    robot.frontLeftMotor.setPower(fl);
                    robot.frontRightMotor.setPower(fr);
                    robot.backLeftMotor.setPower(bl);
                    robot.backRightMotor.setPower(br);

                    // If aligned within tolerance, move to shooter spin-up
                    if (Math.abs(visionAlign.turretPower) < 0.05 &&
                            Math.abs(visionAlign.strafePower) < 0.05 &&
                            Math.abs(visionAlign.drivePower) < 0.05 &&
                            VisionCount == 0 ) {

                        currentState = AutoState.PRELOAD_SPIN_UP;
                        VisionCount++;
                    }
                    else if (Math.abs(visionAlign.turretPower) < 0.05 &&
                            Math.abs(visionAlign.strafePower) < 0.05 &&
                            Math.abs(visionAlign.drivePower) < 0.05 &&
                            VisionCount > 0 ) {

                        currentState = AutoState.SPIN_UP_SHOOTER;
                    }

                    break;

                case PRELOAD_DRIVE_TO_SHOOT:
                    follower.followPath(StartShootPath);
                    currentState = AutoState.VISION_ALIGN;
                    break;

                case PRELOAD_SPIN_UP:
                    if (robot.shooterAtSpeed(RPM_TOLERANCE) && !follower.isBusy()) { // Waits until shooter is at speed
                        robot.feedBall();//Raises feeder
                        stateTimer.reset(); // Allows the state timer to be used in other states
                        currentState = AutoState.PRELOAD_FEED; //Switches state
                    }
                    break;

                case PRELOAD_FEED:
                    if (stateTimer.seconds() > FEED_TIME) { //Waits until feeder has been on long enough
                        robot.prepfeedBall(); // Lowers feeder
                        ballsShot++; // Counts ball

                        if (ballsShot < 3) {
                            currentState = AutoState.PRELOAD_SPIN_UP;
                        } else {
                            ballsShot = 0;
                            buildNewCycle();
                            robot.startIntake();
                            follower.followPath(intakePath);
                            follower.followPath(intakePath2);
                            currentState = AutoState.DRIVE_TO_INTAKE;
                            //Begins driving once all balls have been shot and starts intake motors
                        }
                    }
                    break;

                /* ---------- INTAKE ---------- */

                case DRIVE_TO_INTAKE:
                    if (!follower.isBusy()) { // Path finished
                        robot.stopIntake();
                        robot.setShooterRPM(TARGET_RPM); // Causes shooter to spin while it spins to the shooting position
                        follower.followPath(shootPath);
                        currentState = AutoState.DRIVE_TO_SHOOT;
                    }
                    break;

                case DRIVE_TO_SHOOT:
                    if (!follower.isBusy()) {
                        currentState = AutoState.VISION_ALIGN;
                    } // Waits until robot arrives
                    break;

                /* ---------- SHOOT ---------- */

                case SPIN_UP_SHOOTER:
                    if (robot.shooterAtSpeed(RPM_TOLERANCE)) {
                        robot.feedBall();
                        robot.prepfeedBall();
                        stateTimer.reset();
                        currentState = AutoState.FEED_BALL;
                    }
                    break;

                case FEED_BALL:
                    if (stateTimer.seconds() > FEED_TIME) {
                        robot.resetFeeder();
                        ballsShot++;

                        if (ballsShot < 3) {
                            currentState = AutoState.SPIN_UP_SHOOTER;
                        } else {
                            ballsShot = 0;
                            cycleIndex++;
                            currentState = AutoState.DONE;
                        }
                    }
                    break;

                /* ---------- LOOP ---------- */

                case DONE:
                    if (cycleIndex < MAX_CYCLES) { // Checks if there are more cycles remaining
                        buildNewCycle();
                        robot.startIntake();
                        follower.followPath(intakePath);
                        follower.followPath(intakePath2);
                        currentState = AutoState.DRIVE_TO_INTAKE;
                    } else {
                        robot.stopShooter(); // Ends shooting
                    }
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData("Cycle", cycleIndex);
            telemetry.addData("Pose", follower.getPose());
            telemetry.update();

        }
    }

    /* ---------- PATH BUILDER ---------- */

    private void buildNewCycle() { // Creates paths for the next cycle
        Pose currentPose = follower.getPose(); // Uses the robot's currentpose
        Pose intakePose = intakePoses[cycleIndex]; // selects the right intake target
        Pose intakePose2 = intakePoses2[cycleIndex];

        intakePath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(currentPose, intakePose)))
                .build(); // Builds the path to the intake

        intakePath2 = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(intakePose, intakePose2)))
                .build(); // Builds the path to the end of the intake

        shootPath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(intakePose2, shootPose)))
                .build(); // Same as above but for shooter

        StartShootPath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(startPose, shootPose)))
                .build();

    }

}