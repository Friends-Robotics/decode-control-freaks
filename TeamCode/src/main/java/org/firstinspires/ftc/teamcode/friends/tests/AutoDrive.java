package org.firstinspires.ftc.teamcode.friends.tests;

import com.bylazar.field.Line;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
            goalPose = new Pose(135, 12, 0);

            if(close)
            {
                shootPose = new Pose(90, 60, Math.toRadians(135));
                AutoParkingPose = new Pose(108, 60, Math.toRadians(135));
            }
            else
            {
                shootPose = new Pose(18,60, Math.toRadians(115));
                AutoParkingPose = new Pose(12, 39, Math.toRadians(90));
            }
        } else {
            goalPose = new Pose(135, 132, 0); // don't need angle
            if(close)
            {
                shootPose = new Pose(90, 84, Math.toRadians(45));
                AutoParkingPose = new Pose(109, 80, Math.toRadians(45));
            }
            else
            {
                shootPose = new Pose(18, 84, Math.toRadians(65));
                AutoParkingPose = new Pose(12, 109 ,Math.toRadians(90));
            }
        }
    }



    public void driveToShoot() {
        Pose currentPose = follower.getPose();

        follower.followPath(
                new PathBuilder(follower)
                        .addPath(new Path(new BezierLine(currentPose, shootPose)))
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
