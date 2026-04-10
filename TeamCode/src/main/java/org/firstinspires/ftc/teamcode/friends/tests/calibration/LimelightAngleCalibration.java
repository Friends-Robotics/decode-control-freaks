package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.friends.components.Robot;
import org.firstinspires.ftc.teamcode.friends.components.RobotHardware;

@Config
@TeleOp(name = "Limelight Angle Calibration", group = "Calibration")
public class LimelightAngleCalibration extends LinearOpMode {

    // PHYSICAL MEASUREMENTS (Adjust these in Dashboard)
    public static double KNOWN_DISTANCE_INCHES = 48.0; // Distance from lens to target
    public static double TARGET_HEIGHT_INCHES = 29.5;  // Height of center of target
    public static double CAMERA_HEIGHT_INCHES = 13.75; // Height of Limelight lens

    @Override
    public void runOpMode() {
        RobotHardware robotHardware = new RobotHardware(hardwareMap);
        Robot robot = new Robot(robotHardware);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.shooter.startLimelight(false);

        telemetry.addLine("1. Place robot exactly " + KNOWN_DISTANCE_INCHES + " in from target");
        telemetry.addLine("2. Ensure Limelight sees the target");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = robotHardware.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double ty = result.getTy();

                // Calculate the mounting angle
                // angle = atan((h2-h1) / d) - ty
                double angleRadians = Math.atan2(TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES, KNOWN_DISTANCE_INCHES);
                double calculatedAngleDegrees = Math.toDegrees(angleRadians) - ty;

                telemetry.addData("Status", "Target Found");
                telemetry.addData("Vertical Offset (ty)", ty);
                telemetry.addData("--- CALCULATED MOUNTING ANGLE ---", calculatedAngleDegrees);
                telemetry.addLine("\nPut this value in RobotConstants.Vision.CAMERA_ANGLE");
            } else {
                telemetry.addData("Status", "NO TARGET SEEN");
            }

            telemetry.addData("Known Distance", KNOWN_DISTANCE_INCHES);
            telemetry.update();
        }
    }
}
