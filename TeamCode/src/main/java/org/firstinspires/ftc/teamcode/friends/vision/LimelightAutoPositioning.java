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

    private final double kP_turret = 0.015;
    private final double maxTurretPower = 0.35;
    private final double MaxDrivePower = 0.4;

    private final double DRIVE_TOLERANCE = 0.05;

    private final double kP_turn = 0.02;
    private final double maxTurnPower = 0.4;

    double DESIRED_DISTANCE = 71;
    double DESIRED_DISTANCE_METRES = DESIRED_DISTANCE * 0.0254;

    @Override
    public void runOpMode() {

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0); // Make sure pipeline 0 is AprilTag

        //Hardwaremap assigned
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");

        TurretMotorLeft  = hardwareMap.get(DcMotor.class, "TURRET_L");
        TurretMotorRight = hardwareMap.get(DcMotor.class, "TURRET_R");

        TurretMotorRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            double turretPower = 0;
            double drivePower = 0;

            // ---- AUTO TURRET TRACKING ----
            if (gamepad1.right_bumper && result != null && result.isValid()) {

                Pose3D targetPose = result.getBotpose();

                // Yaw error = how far turret is off target
                double yawError = targetPose.getOrientation().getYaw();

                turretPower = Range.clip(
                        yawError * kP_turret,
                        -maxTurretPower,
                        maxTurretPower
                );

                // ---- AUTO DRIVE TO DISTANCE ----
                double distanceError =
                        targetPose.getPosition().z - DESIRED_DISTANCE_METRES;

                double kP_drive = 0.7;
                drivePower = distanceError * kP_drive;

                // Smooth stop
                if (Math.abs(distanceError) < DRIVE_TOLERANCE) { // 5 cm tolerance
                    drivePower = 0;
                }

                drivePower = Range.clip(drivePower, -0.4, 0.4);
            }

            // ---- SET TURRET MOTORS ----
            TurretMotorLeft.setPower(turretPower);
            TurretMotorRight.setPower(turretPower);

            // ---- DRIVER CONTROLS ----
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double rotate =  gamepad1.right_stick_x; // driver rotation only

            // Override forward/back when auto active
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

