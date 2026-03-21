package org.firstinspires.ftc.teamcode.friends.tests;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.friends.comp.Helpers;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {

    hardwareMap robot;
    Helpers helpers;
    VisionAlign vision;
    ShooterController AutoShoot;
    Limelight3A limelight;
    Follower follower;

    boolean AutoDriveActive = false;
    double drive, strafe, rotate;


    // --- Shooting zones ---

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new hardwareMap(hardwareMap);
        helpers = new Helpers(robot);
        vision = new VisionAlign();
        AutoShoot = new ShooterController();



        limelight = hardwareMap.get(Limelight3A.class, "limelight"); //Make sure to name the Ethernet Device "limelight"
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        //ODOMETRY

        boolean isBlue = false; // set this before match
        boolean close = true; // also set for startingPose depending on what auto is ran

        follower = Constants.createFollower(hardwareMap);
        AutoDrive autoDrive = new AutoDrive(follower, isBlue, close);// this is the same as the parking pose from the last auto
        follower.setStartingPose(autoDrive.getAutoParkingPose());

        Pose goalPose = autoDrive.getGoalPose();

        OdometryShooter odometryShooter = new OdometryShooter(
                0.05,     // kOdoAim
                3300, 4100,   // CLOSE_RPM, FAR_RPM
                60, 130,      // CLOSE_DIST, FAR_DIST
                0.00, 0.25    // CLOSE_HOOD, FAR_HOOD
        );

        //ODOMETRY

        while (opModeIsActive()) {


            // --- Update Everything
            helpers.updateGamepads(gamepad1, gamepad2);
            helpers.readDriveInputs();

            LLResult result = limelight.getLatestResult();
            vision.update(result, true, robot.turretMotor.getCurrentPosition());

            // --- Odometry autoDrive

            if (gamepad1.right_bumper && !AutoDriveActive) {
                autoDrive = new AutoDrive(follower, isBlue, true);
                autoDrive.driveToShoot();
                AutoDriveActive = true;
            }

            if (gamepad1.left_bumper && !AutoDriveActive) {
                autoDrive = new AutoDrive(follower, isBlue, false);
                autoDrive.driveToShoot();
                AutoDriveActive = true;
            }


            // --- Dynamic shooter + hood using odometry
            follower.update();
            Pose robotPose = follower.getPose();

            // calculate distance
            double distance = odometryShooter.getDistance(robotPose, goalPose);

            // calculate RPM & hood
            double targetRPM = odometryShooter.getTargetRPM(distance);
            double hoodPos = odometryShooter.getHoodPosition(distance);

            // calculate turret power
            double turretPower = odometryShooter.getTurretPower(
                    robotPose,
                    goalPose,
                    vision.turretRotatePower,  // vision correction
                    helpers.rotate,
                    helpers.strafe,
                    robot.turretMotor.getCurrentPosition()
            );

            robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;


            // --- AUTO SHOOT --- no A button press
            boolean readyToShoot =
                    vision.isAligned &&
                            robot.shooterAtSpeed(50) &&
                            Math.abs(helpers.rotate) < 0.1 &&
                            Math.abs(helpers.drive) < 0.1 &&
                            Math.abs(helpers.strafe) < 0.1;

            if (!AutoShoot.isBusy() && readyToShoot) {
                AutoShoot.startShooting(3, hoodPos, targetRPM);
            }

            if (AutoDriveActive && gamepad1.right_bumper) {
                follower.update();

                if (!autoDrive.isBusy()) {
                    AutoDriveActive = false;
                }
            } else {
                if (!AutoShoot.isBusy()) {
                    helpers.handleDrive();
                    helpers.handleIntake();
                }
            }

            //AutoRotate
            double autoRotate = OdometryShooter.getDriveRotatePower(robotPose, goalPose);

            if (AutoDriveActive || AutoShoot.isBusy()) {
                helpers.rotate = autoRotate;
            }

            // --- TELEMETRY (FOR TUNING)
            telemetry.addData("Distance", distance);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Hood", hoodPos);


            //Applies override for turret and hood
            boolean manualOverride = gamepad2.dpad_right;

            if (manualOverride) {
                robot.hood.setPosition(0);
                helpers.handleTurret();
                helpers.handleShooterAngle();
            } else {
                robot.turretMotor.setPower(turretPower);
            }

            // --- Shooter state machine ---
            AutoShoot.update(robot, helpers, vision);
            // --- Telemetry ---
            telemetry.addData("Shooter RPM", robot.getShooterRPM());
            telemetry.addData("Hood Pos", AutoShoot.hoodPos);
            telemetry.addData("Turret Power", robot.turretMotor.getPower());
            telemetry.addData("Drive Active", AutoDriveActive);
            telemetry.addData("Vision Ta", result != null ? result.getTa() : 0);
            telemetry.addData("Aligned", vision.isAligned);
            telemetry.addData("Current Robot Pose", follower.getPose());
            telemetry.update();


        }



    }

}
