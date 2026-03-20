package org.firstinspires.ftc.teamcode.friends.tests;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.friends.comp.Comp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {

    hardwareMap robot;
    Comp comp;
    VisionAlign vision;
    ShooterController AutoShoot;
    Limelight3A limelight;
    Follower follower;

    boolean AutoDriveActive = false;

    // --- Shooting zones ---

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new hardwareMap(hardwareMap);
        comp = new Comp();
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
        follower.setStartingPose(autoDrive.AutoParkingPose);

        Pose goalPose = autoDrive.getGoalPose();

        OdometryShooter odometryShooter = new OdometryShooter(
                0.01,     // kOdoAim
                3300, 4100,   // CLOSE_RPM, FAR_RPM
                60, 130,      // CLOSE_DIST, FAR_DIST
                0.00, 0.25    // CLOSE_HOOD, FAR_HOOD
        );

        //ODOMETRY

        while (opModeIsActive()) {


            // --- Update Everything
            comp.updateGamepads();
            comp.readDriveInputs();
            AutoDriveActive = false;

            LLResult result = limelight.getLatestResult();
            vision.update(result, true, robot.turretMotor.getCurrentPosition());

            // --- Odometry autoDrive

            if (comp.currentGp1.right_bumper && !AutoDriveActive) {
                autoDrive = new AutoDrive(follower, isBlue, true);
                autoDrive.driveToShoot();
                AutoDriveActive = true;
            }

            if (comp.currentGp1.left_bumper && !AutoDriveActive) {
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
                    comp.rotate,
                    comp.strafe,
                    robot.turretMotor.getCurrentPosition()
            );

            robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;


            // --- AUTO SHOOT --- no A button press
            boolean readyToShoot =
                    vision.isAligned &&
                            robot.shooterAtSpeed(50) &&
                            Math.abs(comp.rotate) < 0.1 &&
                            Math.abs(comp.drive) < 0.1 &&
                            Math.abs(comp.strafe) < 0.1;

            if (!AutoShoot.isBusy() && readyToShoot) {
                AutoShoot.startShooting(3, hoodPos, targetRPM);
            }

            if (AutoDriveActive && comp.currentGp1.right_bumper && !comp.previousGp1.right_bumper) {
                follower.update();

                if (!autoDrive.isBusy()) {
                    AutoDriveActive = false;
                }
            } else {
                if (!AutoShoot.isBusy()) {
                    comp.applyDrive();
                }
            }

            //AutoRotate
            double autoRotate = OdometryShooter.getDriveRotatePower(robotPose, goalPose);

            if (AutoDriveActive || AutoShoot.isBusy()) {
                comp.rotate = autoRotate;
            }

            // --- TELEMETRY (FOR TUNING)
            telemetry.addData("Distance", distance);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Hood", hoodPos);

            //Intake + drive
            if (!AutoShoot.isBusy()) {
                comp.handleIntake();
                comp.applyDrive();
            }
            //Applies override for turret and hood
            boolean manualOverride = comp.currentGp2.dpad_right;

            if (manualOverride) {
                robot.hood.setPosition(0);
                comp.handleTurret();
                comp.handleShooterAngle();
            } else {
                robot.turretMotor.setPower(turretPower);
            }

            // --- Shooter state machine ---
            AutoShoot.update(robot, comp, vision);
            // --- Telemetry ---
            telemetry.addData("Shooter RPM", robot.getShooterRPM());
            telemetry.addData("Hood Pos", AutoShoot.hoodPos);
            telemetry.addData("Turret Power", robot.turretMotor.getPower());
            telemetry.addData("Drive Active", AutoDriveActive);
            telemetry.addData("Vision Ta", result != null ? result.getTa() : 0);
            telemetry.addData("Aligned", vision.isAligned);
            telemetry.update();


        }

    }

}
