package org.firstinspires.ftc.teamcode.friends.tests;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        comp = new Comp(robot);
        vision = new VisionAlign();
        AutoShoot = new ShooterController();

        //Testing
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        limelight = hardwareMap.get(Limelight3A.class, "limelight"); //Make sure to name the Ethernet Device "limelight"
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        //ODOMETRY

        boolean IsBlue = false; // set this before match
        boolean close = true; // also set for startingPose depending on what auto is ran

        follower = Constants.createFollower(hardwareMap);
        AutoDrive autoDrive = new AutoDrive(follower, IsBlue, close);// this is the same as the parking pose from the last auto
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

            while (opModeIsActive()) {

                // --- Update inputs
                comp.updateGamepads(gamepad1, gamepad2);
                comp.readDriveInputs();

                //Update turret

                // --- Update follower ONCE
                follower.update();
                Pose robotPose = follower.getPose();

                // --- Vision update
                LLResult result = limelight.getLatestResult();
                vision.update(result, true, robot.turretMotor.getCurrentPosition());

                // --- Distance calc
                double distance = odometryShooter.getDistance(robotPose, goalPose);

                // =========================
                // AUTO DRIVE TRIGGERS
                // =========================

                // RIGHT BUMPER → CLOSE (edge detect)
                if (comp.currentGp1.right_bumper && !comp.previousGp1.right_bumper) {

                    // prevent jitter if already close
                    if (distance > 5) {
                        autoDrive = new AutoDrive(follower, IsBlue, true);
                        autoDrive.driveToShoot();
                        AutoDriveActive = true;
                    }
                }

                // LEFT BUMPER → FAR (edge detect)
                if (comp.currentGp1.left_bumper && !comp.previousGp1.left_bumper) {

                    if (distance > 5) {
                        autoDrive = new AutoDrive(follower, IsBlue, false);
                        autoDrive.driveToShoot();
                        AutoDriveActive = true;
                    }
                }

                // =========================
                // STOP AUTO WHEN DONE
                // =========================
                if (AutoDriveActive && !follower.isBusy()) {
                    AutoDriveActive = false;
                }

                // =========================
                // SHOOTER CALCULATIONS
                // =========================

                double targetRPM = odometryShooter.getTargetRPM(distance);
                double hoodPos = odometryShooter.getHoodPosition(distance);

                /*double turretPower = odometryShooter.getTurretPower(
                        robotPose,
                        goalPose,
                        vision.turretRotatePower,
                        comp.rotate,
                        comp.strafe,
                        robot.turretMotor.getCurrentPosition()
                );

                 */

                robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 * targetRPM;

                // =========================
                // AUTO SHOOT
                // =========================
                boolean readyToShoot =
                        vision.isAligned &&
                                Math.abs(comp.rotate) < 0.1 &&
                                Math.abs(comp.drive) < 0.1 &&
                                Math.abs(comp.strafe) < 0.1 &&
                                gamepad1.a;

                if (!AutoShoot.isBusy() && readyToShoot) {
                    AutoShoot.startShooting(3, hoodPos, targetRPM);
                }

                // =========================
                // DRIVE CONTROL (NO FIGHTING)
                // =========================
                if (!AutoDriveActive && !AutoShoot.isBusy()) {
                    comp.applyDrive();
                    comp.handleIntake();
                }

                // =========================
                // AUTO ROTATE (only when useful)
                // =========================
                double autoRotate = OdometryShooter.getDriveRotatePower(robotPose, goalPose);

                if ((AutoDriveActive || AutoShoot.isBusy()) && distance > 5) {
                    comp.rotate = autoRotate;
                }

                // =========================
                // TURRET / HOOD CONTROL
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

}
