package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.friends.components.Intake;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;
import org.firstinspires.ftc.teamcode.friends.components.Shooter;
import org.firstinspires.ftc.teamcode.friends.components.Robot;

@Autonomous
public class AutoClose extends LinearOpMode {

    RobotHardware robotHardware;
    Robot robot;
    Intake intake;
    Shooter shooter;
    private Follower follower;

    int cycleIndex = 0;

    enum AutoState{
        START_TO_SHOOT,
        SHOOTING,
        SHOOT_TO_INTAKE,
        INTAKE_TO_SHOOT,
        PARKING,
        DONE
    }
    AutoState currentState = AutoState.START_TO_SHOOT;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap);
        robot = new Robot(robotHardware);
        follower = Constants.createFollower(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            switch(currentState)
            {
                case START_TO_SHOOT:
                    if(!follower.isBusy())
                    {

                    }
            }
        }


    }
        public static class BuildNewCycle {
            public PathChain StartShootPath;
            public PathChain ShootIntakePath;
            public PathChain IntakePath;
            public PathChain IntakeShootPath;

            Pose[] IntakePose1 = {
                    new Pose(100.000,84.000),
            };
            Pose[] IntakePose2 = {
                    new Pose(127.000,84.000)
            };
            Pose startPose = new Pose(122.500,122.500,37);
            Pose shootPose = new Pose(108.000,107.500,45);
            public BuildNewCycle(Follower follower) {
                StartShootPath = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(startPose.getX(), startPose.getY()),

                                        new Pose(shootPose.getX(), shootPose.getY())
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(startPose.getHeading()), Math.toRadians(shootPose.getHeading()))

                        .build();

                ShootIntakePath = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(108.000, 107.500),

                                        new Pose(100.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                        .build();

                IntakePath = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(100.000, 84.000),

                                        new Pose(127.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                IntakeShootPath = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(127.000, 84.000),

                                        new Pose(108.000, 107.500)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                        .build();
            }
        }

}
