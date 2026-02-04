package org.firstinspires.ftc.teamcode.friends.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.friends.hardwareMap;
@TeleOp
public class LimelightAutoPositioning extends LinearOpMode {

    public Limelight3A limelight;

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    private final double kP_turn = 0.02;
    private final double maxTurnPower = 0.4;

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

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            double turnPower = 0.4;
            double drivePower = 0.6;

            // Target distance in meters
            double targetDistanceMeters = 135.5 * 0.0254; // 135.5 inches -> meters

            // Auto-turn and auto-drive logic
            if (gamepad1.right_bumper && result != null && result.isValid()) {
                Pose3D targetPose = result.getBotpose();

                // ---- Auto-turn ----
                double yawError = targetPose.getOrientation().getYaw(); // degrees
                turnPower = Range.clip(yawError * kP_turn, -maxTurnPower, maxTurnPower);

                // ---- Auto-drive with smooth stopping ----
                double distanceError = targetPose.getPosition().z - targetDistanceMeters; // meters
                double kP_drive = 0.8; // proportional gain for speed
                drivePower = distanceError * kP_drive;

                // Smooth stop: reduce power when close to target
                double minDistanceForStop = 0.05; // 5 cm tolerance
                if (Math.abs(distanceError) < minDistanceForStop) {
                    drivePower = 0; // close enough, stop
                } else {
                    // Scale speed down as robot gets closer (optional)
                    drivePower = Range.clip(drivePower, -0.4, 0.4); // max forward/back power
                }
            }

            // ---- Driver inputs ----
            double drive = -gamepad1.left_stick_y;   // forward/back
            double strafe = gamepad1.left_stick_x;   // left/right
            double rotate = gamepad1.right_stick_x;  // normal rotation

            // Override with auto-turn/drive if active
            if (gamepad1.right_bumper && result != null && result.isValid()) {
                rotate = turnPower;
                drive = drivePower;
            }

            // ---- Mecanum drive calculations ----
            double flPower = drive + strafe + rotate;
            double frPower = drive - strafe - rotate;
            double blPower = drive - strafe + rotate;
            double brPower = drive + strafe - rotate;

            // Clip powers to [-1, 1]
            flPower = Range.clip(flPower, -1, 1);
            frPower = Range.clip(frPower, -1, 1);
            blPower = Range.clip(blPower, -1, 1);
            brPower = Range.clip(brPower, -1, 1);

            // Set motor powers
            frontLeftMotor.setPower(flPower);
            frontRightMotor.setPower(frPower);
            backLeftMotor.setPower(blPower);
            backRightMotor.setPower(brPower);

            // ---- Telemetry ----
            telemetry.addData("Auto Active", gamepad1.right_bumper && result != null && result.isValid());
            if (result != null && result.isValid()) {
                Pose3D targetPose = result.getBotpose();
                telemetry.addData("Distance (m)", targetPose.getPosition().z);
                telemetry.addData("Yaw error (deg)", targetPose.getOrientation().getYaw());
                telemetry.addData("Drive Power", drivePower);
                telemetry.addData("Turn Power", turnPower);
            } else {
                telemetry.addLine("No valid target");
            }
            telemetry.update();//d
        }


    }
}

