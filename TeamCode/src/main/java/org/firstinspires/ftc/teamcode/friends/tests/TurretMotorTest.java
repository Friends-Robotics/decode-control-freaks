package org.firstinspires.ftc.teamcode.friends.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.friends.comp.TeamHardwareMap;

public class TurretMotorTest extends LinearOpMode {

    TeamHardwareMap robot;
    //ticksPerDegree =
    //(motorTicksPerOutputRev × externalGearRatio) ÷ 360
    //28 x 5 = 140ticks per bearing revolution


    @Override
    public void runOpMode() { //Angle = 360



        // Reset and enable encoder
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        robot.turretMotor.setPower(0);

        while (opModeIsActive()) {
            telemetry.addData("Encoder Ticks", robot.turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }

}
