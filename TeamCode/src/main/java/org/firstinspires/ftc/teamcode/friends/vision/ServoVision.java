package org.firstinspires.ftc.teamcode.friends.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

// Vision based on a servo attached to the webcam which turns

@TeleOp(name = "Vision Test")
public class ServoVision extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Servo servo;
    final double outerServoChange = 0.005;
    final double innerServoChange = 0.001;
    private boolean lastLookedRight = false;

    @Override
    public void runOpMode() {

        initAprilTag();
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                processCameraOutput();

                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(435.14, 436.58, 323.26, 162.08)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(aprilTag)
                .build();


        servo = hardwareMap.get(Servo.class, "Servo");
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(0.5);
    }
    private void processCameraOutput() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if(currentDetections.isEmpty()) {
            lookForAprilTags();
            return;
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) continue;
            if(detection.id == 20 || detection.id == 24) // 2 Basket aprilTag id's
                processAprilTag(detection);
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
    }
    private void processAprilTag(AprilTagDetection detection){
        telemetry.addLine(String.format(Locale.ENGLISH ,"\n==== (ID %d) %s", detection.id, detection.metadata.name));
        telemetry.addLine(String.format(Locale.ENGLISH ,"XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));

        if(detection.ftcPose.x > 1) { //Some arbitrary number
            servo.setPosition(servo.getPosition() + innerServoChange);
            telemetry.addLine("TURNING LEFT");
        }
        else if(detection.ftcPose.x < -1){ //The negative of that arbitrary number
            servo.setPosition(servo.getPosition() - innerServoChange);
            telemetry.addLine("TURNING RIGHT");
        }
    }
    private void lookForAprilTags(){
        telemetry.addLine("LOOKING FOR APRILTAGS");

        if(lastLookedRight){
            servo.setPosition(servo.getPosition() - outerServoChange);
        }
        else{
            servo.setPosition(servo.getPosition() + outerServoChange);
        }

        if(servo.getPosition() == 0 || servo.getPosition() == 1) lastLookedRight = !lastLookedRight;
    }
}