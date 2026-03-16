package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Pinpoint Test")
public class PinpointTest extends LinearOpMode {

    Follower follower;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        follower.setPose(new Pose(0,0,0));

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            Pose pose = follower.getPose();

            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));

            telemetry.update();
        }
    }
}
//Push robot by hand to check if pinpoint is working