package org.firstinspires.ftc.teamcode.friends.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.Path;

public class AutoDrive {
    private final Follower follower;

    private final Pose goalPose;
    private final Pose shootPose;
    private final Pose AutoParkingPose;




    public AutoDrive(Follower follower, boolean isBlue, boolean close) {
        // y --> x ^ reverse from pedropathing visualizer
        this.follower = follower;

        if (isBlue) {
            goalPose = new Pose(12, 135, 0);

            if(close)
            {
                shootPose = new Pose(60, 90, Math.toRadians(135));
                AutoParkingPose = new Pose(60, 108, Math.toRadians(90));
            }
            else
            {
                shootPose = new Pose(60,18, Math.toRadians(115));
                AutoParkingPose = new Pose(39, 12, Math.toRadians(90));
            }
        } else {
            goalPose = new Pose(134, 133, 0); // don't need angle
            if(close)
            {
                shootPose = new Pose(84, 90, Math.toRadians(45));
                AutoParkingPose = new Pose(80, 109, Math.toRadians(90));
            }
            else
            {
                shootPose = new Pose(84, 18, Math.toRadians(65));
                AutoParkingPose = new Pose(109, 12 ,Math.toRadians(90));
            }
        }
    }
    public void driveToShoot() {
        Pose currentPose = follower.getPose();

        follower.followPath(
                new PathBuilder(follower)
                        .addPath(new BezierLine(currentPose, shootPose))
                        .setLinearHeadingInterpolation(
                                currentPose.getHeading(),
                                shootPose.getHeading()
                        )
                        .build()
        );
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public Pose getGoalPose() {
        return goalPose;
    }

    public Pose getShootPose() {
        return shootPose;
    }
    public Pose getAutoParkingPose(){
        return AutoParkingPose;
    }


}
