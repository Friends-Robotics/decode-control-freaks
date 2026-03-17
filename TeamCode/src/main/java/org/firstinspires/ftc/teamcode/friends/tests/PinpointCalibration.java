package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Calibration", group = "Calibration")
public class PinpointCalibration extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry.addLine("Pinpoint Ready");
        telemetry.addLine("Press START to calibrate");
        telemetry.update();

        waitForStart();

        // Reset position & calibration
        pinpoint.resetPosAndIMU();

        // -------- STEP 1 --------
        telemetry.addLine("STEP 1: Keep robot STILL");
        telemetry.update();
        sleep(3000);

        // -------- STEP 2 --------
        telemetry.addLine("STEP 2: Push robot FORWARD 24-48 inches");
        telemetry.update();
        sleep(5000);

        // -------- STEP 3 --------
        telemetry.addLine("STEP 3: Push robot SIDEWAYS 24-48 inches");
        telemetry.update();
        sleep(5000);

        // -------- STEP 4 --------
        telemetry.addLine("STEP 4: Rotate robot slowly 360 degrees");
        telemetry.update();
        sleep(7000);

        telemetry.addLine("Calibration Done!");
        telemetry.update();
        sleep(2000);

        // -------- LIVE DATA --------
        while (opModeIsActive()) {

            telemetry.addData("X (in)", pinpoint.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", Math.toDegrees(pinpoint.getHeading(AngleUnit.RADIANS)));

            telemetry.update();
        }
    }
}