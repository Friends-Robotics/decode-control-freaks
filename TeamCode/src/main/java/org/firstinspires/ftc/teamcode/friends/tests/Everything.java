package org.firstinspires.ftc.teamcode.friends.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); // this is the same as the parking pose from the last auto

        OdometryShooter odometryShooter = new OdometryShooter(
                0.01,     // kOdoAim
                3300, 4100,   // CLOSE_RPM, FAR_RPM
                60, 130,      // CLOSE_DIST, FAR_DIST
                0.00, 0.25    // CLOSE_HOOD, FAR_HOOD
        );

        // --- GOAL POSITION
        Pose goalPose = new Pose(134, 139, 0); // example


        //ODOMETRY

        while (opModeIsActive()) {

            // --- Update driver gamepads
            comp.updateGamepads();
            comp.readDriveInputs();
            AutoDriveActive = false;

            // --- Vision processing (secondary)

            LLResult result = limelight.getLatestResult();
            vision.update(result, true, robot.turretMotor.getCurrentPosition());

            // --- Determine auto-drive target ---
            if (comp.currentGp1.right_bumper && result != null && result.isValid()) {
                comp.drive = vision.drivePowerClose;
                comp.rotate = 0;
                AutoDriveActive = true;
            } else if (comp.currentGp1.left_bumper && result != null && result.isValid()) {
                comp.drive = vision.drivePowerFar;
                comp.rotate = 0;
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
                    comp.strafe
            );

            robot.turretMotor.setPower(turretPower);
            robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;


            // --- AUTO SHOOT ---
            if (!AutoShoot.isBusy()
                    && comp.currentGp1.a
                    && vision.isAligned
                    && robot.shooterAtSpeed(50)
                    && Math.abs(comp.rotate) < 0.1) {

                AutoShoot.startShooting(3, hoodPos, targetRPM);
            }

            // --- TELEMETRY (FOR TUNING)
            telemetry.addData("Distance", distance);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Hood", hoodPos);

            //Intake + drive
            if(!AutoShoot.isBusy())
            {
                comp.handleIntake();
                comp.applyDrive();
            }
            //Applies override for turret and hood
            if(comp.currentGp2.dpad_right && !comp.previousGp2.dpad_right)
            {
                robot.turretMotor.setPower(0);
                robot.hood.setPosition(0);
                comp.handleTurret();
                comp.handleShooterAngle();
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

    // --- Simple odometry auto-drive ---


}
