package org.firstinspires.ftc.teamcode.friends.comp;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.tests.ShooterController;
import org.firstinspires.ftc.teamcode.friends.tests.OdometryShooter;
import org.firstinspires.ftc.teamcode.friends.tests.AutoDrive;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "3BallStrafeAuto")
public class BallAuto3 extends LinearOpMode {

    hardwareMap robot;
    Follower follower;

    ShooterController shooterController;
    VisionAlign vision;

    PathChain strafePath;
    PathChain reversePath;

    Pose startPose;

    @Override
    public void runOpMode() {

        robot = new hardwareMap(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        shooterController = new ShooterController();
        vision = new VisionAlign();

        startPose= new Pose(0,0,0);
        follower.setStartingPose(startPose);

        telemetry.addLine("3 Ball Strafe Auto Ready");
        telemetry.update();
        waitForStart();

        // Step 1 — drive back once
        buildReversePath();
        follower.followPath(reversePath);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine("Reversing...");
            telemetry.update();
        }

        // Step 2 — shoot 3 balls
        shooterController.startShooting(3, 0, 3100);
        while (opModeIsActive() && shooterController.isBusy()) {
            shooterController.update(robot, null, vision);
            telemetry.addLine("Shooting...");
            telemetry.update();
        }

        sleep(300);

        // Step 3 — strafe
        buildStrafePath();
        follower.followPath(strafePath);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine("Strafing...");
            telemetry.update();
        }

        robot.stopShooter();
    }

    // ---------- STRAFE LEFT ----------
    private void buildStrafePath() {

        Pose currentPose = follower.getPose();

        Pose strafePose = new Pose(
                currentPose.getX(),
                currentPose.getY() + 24, // <-- adjust distance
                currentPose.getHeading()
        );

        strafePath = new PathBuilder(follower)
                .addPath(new Path(new BezierLine(currentPose, strafePose)))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        strafePose.getHeading()
                )
                .build();
    }

    private void buildReversePath(){

        Pose currentPose = follower.getPose();

        Pose reversePose = new Pose(
                currentPose.getX() - 12, // Change sign to plus for blue
                currentPose.getY() - 24, // <-- adjust distance
                currentPose.getHeading()
        );

        reversePath = new PathBuilder(follower)
                .addPath(new Path(new BezierLine(currentPose, reversePose)))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        reversePose.getHeading()
                )
                .build();
    }

}