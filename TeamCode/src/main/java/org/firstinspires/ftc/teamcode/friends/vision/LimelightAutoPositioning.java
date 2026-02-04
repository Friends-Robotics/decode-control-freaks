package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class LimelightAutoPositioning extends LinearOpMode {

    public Limelight3A limelight;

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor TurretMotorLeft;
    public DcMotor TurretMotorRight;

    private final double kP_strafe = 0.6;
    private final double kP_turret = 0.015; // A proportional constant for turret aiming and controls how fast the turret reacts to angle error
    private final double MaxTurretPower = 0.35; //Limits how fast the turret can spin
    private final double MaxDrivePower = 0.4;
    private final double MaxStrafePower = 0.4;

    private final double DRIVE_TOLERANCE = 0.05; //stops the drivetrain if its within a certain range
    private final double TURRET_TOLERANCE = 1;//Degrees
    private final double STRAFE_TOLERANCE = 0.03;


    double DESIRED_DISTANCE = 71;
    private final double DESIRED_DISTANCE_METRES = DESIRED_DISTANCE * 0.0254;

    @Override
    public void runOpMode() {

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Controls the fps
        limelight.start();
        limelight.pipelineSwitch(0); // Make sure pipeline 0 is AprilTag

        //Hardwaremap assigned
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");

        TurretMotorLeft  = hardwareMap.get(DcMotor.class, "TURRET_L");
        TurretMotorRight = hardwareMap.get(DcMotor.class, "TURRET_R");

        TurretMotorRight.setDirection(DcMotor.Direction.REVERSE); //Stops motors from working against each other

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            double turretPower = 0; //Resets power for every loop
            double drivePower = 0;
            double strafePower = 0;

            Pose3D targetPose = null; //So i can use the pose outside of the auto tracking if

            // ---- AUTO STRAFE TO CENTER ----
            double xError = targetPose.getPosition().x; // left/right offset

            if (Math.abs(xError) < STRAFE_TOLERANCE) {
                strafePower = 0;
            } else {
                strafePower = Range.clip(
                        xError * kP_strafe,
                        -MaxStrafePower,
                        MaxStrafePower
                );
            }

            // ---- AUTO TURRET TRACKING ----
            if (gamepad1.right_bumper && result != null && result.isValid()) { // If Jonny holds bumper, limelight sees an apriltag and its the right one it will work

                targetPose = result.getBotpose(); //Limelight magic so the robot knows how far it is from the tag

                // Yaw error = how far turret is off april tag target
                double yawError = targetPose.getOrientation().getYaw();

                if (Math.abs(yawError) < TURRET_TOLERANCE) {
                    turretPower = 0;
                } else {
                    turretPower = Range.clip(
                            yawError * kP_turret,
                            -MaxTurretPower,
                            MaxTurretPower
                    );
                }
                // ---- AUTO DRIVE TO DISTANCE ----
                double distanceError = //Positive = Far Negative = close 0 = perrrfect
                        targetPose.getPosition().z - DESIRED_DISTANCE_METRES;

                double kP_drive = 0.7;
                drivePower = distanceError * kP_drive;

                // Smooth stop
                if (Math.abs(distanceError) < DRIVE_TOLERANCE) { // Stops movement when close enough
                    drivePower = 0;
                }

                drivePower = Range.clip(drivePower, -MaxDrivePower, MaxDrivePower);// Keeps movement from blowing up
            }

            // ---- SET TURRET MOTORS ----
            TurretMotorLeft.setPower(turretPower);
            TurretMotorRight.setPower(turretPower);

            // ---- DRIVER CONTROLS ----
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.right_bumper && result != null && result.isValid()) {
                drive = drivePower;
                strafe = strafePower;
                rotate = 0; // lock robot rotation
            }

            // Override forward/back when auto active so it stays within rules
            if (gamepad1.right_bumper && result != null && result.isValid()) {
                drive = drivePower;
            }

            // ---- MECANUM DRIVE ----
            double fl = drive + strafe + rotate;
            double fr = drive - strafe - rotate;
            double bl = drive - strafe + rotate;
            double br = drive + strafe - rotate;

            fl = Range.clip(fl, -1, 1);
            fr = Range.clip(fr, -1, 1);
            bl = Range.clip(bl, -1, 1);
            br = Range.clip(br, -1, 1);

            frontLeftMotor.setPower(fl);
            frontRightMotor.setPower(fr);
            backLeftMotor.setPower(bl);
            backRightMotor.setPower(br);

            // ---- TELEMETRY ----
            telemetry.addData("Auto Active", gamepad1.right_bumper);
            if (result != null && result.isValid()) {
                Pose3D p = result.getBotpose();
                telemetry.addData("Distance (m)", p.getPosition().z);
                telemetry.addData("Yaw Error (deg)", p.getOrientation().getYaw());
                telemetry.addData("Turret Power", turretPower);
                telemetry.addData("Drive Power", drivePower);
            } else {
                telemetry.addLine("No AprilTag");
            }

            telemetry.update();
        }

    }
}

