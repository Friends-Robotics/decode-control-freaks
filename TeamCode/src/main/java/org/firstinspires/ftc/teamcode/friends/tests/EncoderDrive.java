package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MecanumVelocitySync", group="Drive")
public class EncoderDrive extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;

    // Proportional gain (tune this)
    double kP = 0.0004;

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotorEx.class, "FLM");
        fr = hardwareMap.get(DcMotorEx.class, "FRM");
        bl = hardwareMap.get(DcMotorEx.class, "BLM");
        br = hardwareMap.get(DcMotorEx.class, "BRM");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse motors if needed
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            /*
            fl.setPower(0.3);
            fr.setPower(0.3);
            bl.setPower(0.3);
            br.setPower(0.3);

            telemetry.addData("FL Pos: ", fl.getCurrentPosition());
            telemetry.addData("FR Pos: ", fr.getCurrentPosition());
            telemetry.addData("BL Pos: ", bl.getCurrentPosition());
            telemetry.addData("BR Pos: ", br.getCurrentPosition());

            if(gamepad1.touchpad){
                telemetry.addData("FL Vel: ", fl.getVelocity());
                telemetry.addData("FR Vel: ", fr.getVelocity());
                telemetry.addData("BL Vel: ", bl.getVelocity());
                telemetry.addData("BR Vel: ", br.getVelocity());
            }

            telemetry.update();
            */


            // Joystick inputs
            double y  = -gamepad1.left_stick_y;  // forward
            double x  =  gamepad1.left_stick_x;  // strafe
            double rx =  gamepad1.right_stick_x; // rotate

            // Mecanum kinematics (desired powers)
            double fl_d = y + x + rx;
            double fr_d = y - x - rx;
            double bl_d = y - x + rx;
            double br_d = y + x - rx;

            // Normalize
            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(fl_d), Math.abs(fr_d)),
                    Math.max(Math.abs(bl_d), Math.abs(br_d))
            ));

            fl_d /= max;
            fr_d /= max;
            bl_d /= max;
            br_d /= max;

            // Actual velocities (ticks/sec)
            double fl_v = fl.getVelocity();
            double fr_v = fr.getVelocity();
            double bl_v = bl.getVelocity();
            double br_v = br.getVelocity();

            // Convert desired power → desired velocity scale
            // (relative matching, not absolute RPM control)
            double scale = 2000; // approx ticks/sec at full power (tune if needed)

            double fl_target = fl_d * scale;
            double fr_target = fr_d * scale;
            double bl_target = bl_d * scale;
            double br_target = br_d * scale;

            // Errors
            double fl_err = fl_target - fl_v;
            double fr_err = fr_target - fr_v;
            double bl_err = bl_target - bl_v;
            double br_err = br_target - br_v;

            // Corrections
            double fl_c = fl_err * kP;
            double fr_c = fr_err * kP;
            double bl_c = bl_err * kP;
            double br_c = br_err * kP;

            // Final powers
            fl.setPower(fl_d + fl_c);
            fr.setPower(fr_d + fr_c);
            bl.setPower(bl_d + bl_c);
            br.setPower(br_d + br_c);

            telemetry.addLine("Velocities");
            telemetry.addData("FL", fl_v);
            telemetry.addData("FR", fr_v);
            telemetry.addData("BL", bl_v);
            telemetry.addData("BR", br_v);
            telemetry.update();
        }
    }
}
