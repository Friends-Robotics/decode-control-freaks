package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Pinpoint IMU Reset", group = "Calibration")
public class IMUArenaPrep extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // Use the same hardware map name from your PinpointConstants
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // Match your Constants encoder config so everything is consistent
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        telemetry.addLine("IMU recalibration requested...");
        telemetry.addLine("Keep robot STILL on a flat surface.");
        telemetry.update();

        // Recalibrate — robot must be stationary
        pinpoint.resetPosAndIMU();
        sleep(1000);

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();

            if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                telemetry.addLine(">> IMU Recalibration COMPLETE");
            } else if (status == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
                telemetry.addLine(">> Calibrating... keep robot still");
            } else {
                telemetry.addData("Status", status);
            }

            telemetry.addData("Heading (deg)",
                    pinpoint.getPosition());
            telemetry.addLine("\nPress STOP when done.");
            telemetry.update();
        }
    }
}
