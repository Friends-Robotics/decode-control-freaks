package org.firstinspires.ftc.teamcode.friends.tests;

import android.graphics.Path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
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

    boolean AutoDriveActive = false;


    // --- Shooting zones ---
    class ShootPose {
        public double x, y, heading;
        public ShootPose(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }
    ShootPose closeZone = new ShootPose(24, 48, 45); // tune for field
    ShootPose farZone   = new ShootPose(72, 48, 45); // tune for field

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Init ---
        robot = new hardwareMap(hardwareMap);
        comp = new Comp();
        vision = new VisionAlign();
        AutoShoot = new ShooterController();

        limelight = hardwareMap.get(Limelight3A.class, "limelight"); //Make sure to name the Ethernet Device "limelight"
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        //ODOMETRY

        while (opModeIsActive()) {

            // --- Update driver gamepads ---
            comp.updateGamepads();
            comp.readDriveInputs();

            // --- Vision processing ---

            LLResult result = limelight.getLatestResult();
            vision.update(result, true, robot.turretMotor.getCurrentPosition());

            // --- Determine auto-drive target ---
            if (comp.currentGp1.right_bumper && result != null && result.isValid()) {
                comp.drive = vision.drivePowerClose;
                comp.strafe = 0;
                comp.rotate = 0;
                AutoDriveActive = true;
            } else if (comp.currentGp1.left_bumper && result != null && result.isValid()) {
                comp.drive = vision.drivePowerFar;
                comp.strafe = 0;
                comp.rotate = 0;
                AutoDriveActive = true;
            }
            comp.applyDrive();

            // --- Dynamic shooter + hood using vision ---
            if (result != null && result.isValid()) {
                double ta = result.getTa();

                // Dynamic shooter velocity
                double CLOSE_TA = 1.14, FAR_TA = 0.3162;
                double CLOSE_VEL = 3300, FAR_VEL = 4100;
                double slopeVel = (FAR_VEL - CLOSE_VEL) / (FAR_TA - CLOSE_TA);
                double targetVelocity = CLOSE_VEL + slopeVel * (ta - CLOSE_TA);
                robot.targetShooterRPM = 0.8 * robot.targetShooterRPM + 0.2 *
                        Range.clip(targetVelocity, 2500, 5000);

                // Dynamic hood
                double CLOSE_HOOD = 0, FAR_HOOD = 0.25;
                double slopeHood = ((FAR_HOOD - CLOSE_HOOD) / (FAR_TA - CLOSE_TA));
                double hoodPos = Range.clip(CLOSE_HOOD + slopeHood * (ta - CLOSE_TA), 0, 0.3);

                // Alignment check
                vision.isAligned = Math.abs(vision.lastXError) < 1.5;

                // Auto start shooting if driver holds A
                if (!AutoShoot.isBusy() && comp.currentGp1.a) {
                    AutoShoot.startShooting(3, hoodPos);
                }

                // Turret motion compensation
                double turretPower = vision.turretRotatePower
                        + comp.rotate * 0.25
                        + comp.strafe * 0.15;
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

    // --- Simple odometry auto-drive ---


}
