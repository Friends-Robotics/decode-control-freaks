package org.firstinspires.ftc.teamcode.friends.tests;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.friends.comp.Helpers;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {

    hardwareMap robot;
    Helpers comp;
    VisionAlign vision;
    ShooterController AutoShoot;
    Limelight3A limelight;
    Follower follower;
    GoBildaPinpointDriver pinpoint;



    boolean AutoDriveActive = false;


    // --- Shooting zones ---

    @Override
    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        robot = new hardwareMap(hardwareMap);
        comp = new Helpers(robot);
        vision = new VisionAlign();
        AutoShoot = new ShooterController();

        //Testing
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pinpoint.resetPosAndIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight"); //Make sure to name the Ethernet Device "limelight"
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        //ODOMETRY

        pinpoint.resetPosAndIMU();

        boolean IsBlue = false; // set this before match
        boolean close = true; // also set for startingPose depending on what auto is ran

        follower = Constants.createFollower(hardwareMap);
        AutoDrive autoDrive = new AutoDrive(follower, IsBlue, close);// this is the same as the parking pose from the last auto
        follower.setStartingPose(autoDrive.getAutoParkingPose());


        Pose goalPose = autoDrive.getGoalPose();

        OdometryShooter odometryShooter = new OdometryShooter(
                6 , 2
        );

        //ODOMETRY

        while (opModeIsActive()) {

            // =========================
            // UPDATE INPUTS
            // =========================
            comp.updateGamepads(gamepad1, gamepad2);
            comp.readDriveInputs();

            // =========================
            // UPDATE ODOMETRY
            // =========================
            follower.update();
            Pose robotPose = follower.getPose();

            // =========================
            // VISION + TURRET AIMING
            // =========================
            LLResult result = limelight.getLatestResult();

            // Get dynamic turret angle (odometry + goal offset)
            double targetAngle = odometryShooter.getTargetTurretAngle(robotPose, goalPose);

            // VisionAlign = main controller
            vision.update(
                    result,
                    true,
                    robot.turretMotor.getCurrentPosition(),
                    targetAngle
            );

            // Apply turret power
            robot.turretMotor.setPower(vision.turretRotatePower);

            // =========================
            // DISTANCE + SHOOTER CALC
            // =========================
            double distance = odometryShooter.getDistanceToGoal(robotPose, goalPose);

            double targetRPM = odometryShooter.getTargetRPM(
                    distance,
                    3300, 4100,
                    60, 130
            );

            double hoodPos = odometryShooter.getHoodPosition(
                    distance,
                    0.00, 0.25,
                    60, 130
            );

            // Smooth RPM
            robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;

            // =========================
            // AUTO DRIVE TRIGGERS
            // =========================
            if (comp.currentGp1.right_bumper && !comp.previousGp1.right_bumper) {
                if (distance > 5) {
                    autoDrive = new AutoDrive(follower, IsBlue, true);
                    autoDrive.driveToShoot();
                    AutoDriveActive = true;
                }
            }

            if (comp.currentGp1.left_bumper && !comp.previousGp1.left_bumper) {
                if (distance > 5) {
                    autoDrive = new AutoDrive(follower, IsBlue, false);
                    autoDrive.driveToShoot();
                    AutoDriveActive = true;
                }
            }

            if (AutoDriveActive && !follower.isBusy()) {
                AutoDriveActive = false;
            }

            // =========================
            // AUTO SHOOT LOGIC
            // =========================
            boolean readyToShoot =
                    vision.isAligned &&
                            Math.abs(comp.rotate) < 0.1 &&
                            Math.abs(comp.drive) < 0.1 &&
                            Math.abs(comp.strafe) < 0.1 &&
                            Math.abs(vision.turretRotatePower) < 0.03 &&
                            gamepad1.a;

            if (!AutoShoot.isBusy() && readyToShoot) {
                AutoShoot.startShooting(3, hoodPos, targetRPM);
            }

            // =========================
            // DRIVE CONTROL
            // =========================
            if (!AutoDriveActive && !AutoShoot.isBusy()) {
                comp.applyDrive();
                comp.handleIntake();
            }

            // =========================
            // MANUAL OVERRIDE
            // =========================
            boolean manualOverride = gamepad2.dpad_right;

            if (manualOverride) {
                robot.hood.setPosition(0);
                comp.handleTurret();
                comp.handleShooterAngle();
            } else {
                robot.turretMotor.setPower(vision.turretRotatePower);
            }

            // =========================
            // SHOOTER STATE MACHINE
            // =========================
            AutoShoot.update(robot, comp, vision);

            // =========================
            // TELEMETRY
            // =========================
            telemetry.addData("Distance", distance);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Hood", hoodPos);
            telemetry.addData("Shooter RPM", robot.getShooterRPM());
            telemetry.addData("Turret Power", robot.turretMotor.getPower());
            telemetry.addData("Drive Active", AutoDriveActive);
            telemetry.addData("Vision Ta", result != null ? result.getTa() : 0);
            telemetry.addData("Aligned", vision.isAligned);
            telemetry.addData("Pose", robotPose);
            telemetry.update();
        }
    }
}
