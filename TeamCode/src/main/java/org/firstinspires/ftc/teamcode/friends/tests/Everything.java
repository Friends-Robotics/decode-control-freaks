package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.friends.hardwareMap;
import org.firstinspires.ftc.teamcode.friends.vision.VisionAlign;
import org.firstinspires.ftc.teamcode.friends.comp.Comp;


@TeleOp(name = "Drive + Intake + Shooting")
public class Everything extends LinearOpMode {

    //CALLING CLASSES
    hardwareMap robot;
    VisionAlign vision;
    ShooterController AutoShoot;
    Comp comp;

    //DRIVING VARIABLES
    double speedModifier = 0.8;





    @Override
    public void runOpMode() throws InterruptedException{

        robot = new hardwareMap(hardwareMap);
        AutoShoot = new ShooterController();
        vision = new VisionAlign();

        while (opModeIsActive()) {

            comp.updateGamepads();
            comp.sendTelemetry();

            //Vision + ALIGN
            double drives  = -comp.currentGp1.left_stick_y;
            double strafes =  comp.currentGp1.left_stick_x;
            double rotates =  comp.currentGp1.right_stick_x;

            if(!AutoShoot.isBusy() || !comp.currentGp2.dpad_right )
            {
                robot.turretMotor.setPower(vision.turretRotatePower);
            }
            if(!AutoShoot.isBusy() || !comp.currentGp2.dpad_right && comp.currentGp1.right_bumper)
            {
                drives = vision.drivePowerClose;
                strafes = 0;
                rotates = 0;
            }

            // Driver controls
            if(!AutoShoot.isBusy() || !comp.currentGp2.dpad_right){
                comp.handleIntake();
                comp.handleDrive();
                comp.handleTurret();
            }



        }
    }

}
