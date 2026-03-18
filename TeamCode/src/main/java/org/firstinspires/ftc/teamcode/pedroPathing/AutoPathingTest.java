package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;

@Autonomous(name = "Pedro Pinpoint Drive Test")
public class AutoPathingTest extends LinearOpMode {

    hardwareMap robot;
    Follower follower;

    PathChain testPath;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- Initialize ----------
        robot = new hardwareMap(hardwareMap);

        // Refers to constants file
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose (field coordinates)
        Pose startPose = new Pose(0, 0, 0);
        follower.setPose(startPose);

        // Build a simple test path
        Pose midPose = new Pose(24, 0, 0);   // move 24 inches forward
        Pose endPose = new Pose(24, 24, Math.toRadians(90)); // then strafe right 24 inches and rotate 90 degrees

        testPath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(startPose, midPose)))
                .addPath(new Path(new BezierCurve(midPose, endPose)))
                .build();

        telemetry.addLine("Pedro Drive Test Ready");
        telemetry.update();

        waitForStart();

        // ---------- Start following path ----------
        follower.followPath(testPath);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // ---------- Telemetry ----------
            Pose pose = follower.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Following Path?", follower.isBusy());
            telemetry.update();
        }

        // Stop motors after path complete
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        telemetry.addLine("Path Complete");
        telemetry.update();
    }
}
