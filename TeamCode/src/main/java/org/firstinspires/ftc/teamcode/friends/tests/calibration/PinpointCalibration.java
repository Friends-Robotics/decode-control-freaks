package org.firstinspires.ftc.teamcode.friends.tests.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Calibration", group = "Calibration")
public class PinpointCalibration extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Link telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Pinpoint Ready");
        telemetry.addLine("Press START to calibrate");
        telemetry.update();

        waitForStart();

        // Reset position & calibration
        pinpoint.resetPosAndIMU();
        pinpoint.getPosition();;

        while (opModeIsActive()) {
            telemetry.addData("X (in)", pinpoint.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", Math.toDegrees(pinpoint.getHeading(AngleUnit.RADIANS)));

            telemetry.update();
        }
    }
}