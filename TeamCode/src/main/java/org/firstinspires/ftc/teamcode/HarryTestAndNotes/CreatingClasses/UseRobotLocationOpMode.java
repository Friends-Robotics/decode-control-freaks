package org.firstinspires.ftc.teamcode.HarryTestAndNotes.CreatingClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// This OpMode demonstrates using the RobotLocationRadians class
@TeleOp
public class UseRobotLocationOpMode extends OpMode {
    // Create a robot location, starting angle = 0 degrees
    RobotLocationRadians robotLocation = new RobotLocationRadians(0.0);

    @Override
    public void init(){
        // Explicitly set angle to 0 degrees (converted to radians internally)
        robotLocation.setAngle(0);
        // The reason we call setAngle() here is in case we select the OpMode, init it
        // run it and then stop and press init again. If we don't set in init() then it
        // Will keep its value from the last time it was modified
    }

    @Override
    public void loop(){
        // If A is pressed, turn the robot +0.1 degrees
        // (converted to radians internally and added)
        if(gamepad1.a){
            robotLocation.turn(0.1);
        }
        // If B is pressed, turn the robot -0.1 degrees
        else if(gamepad1.b){
            robotLocation.turn(-0.1);
        }
        else if(gamepad1.dpad_left){
            robotLocation.changeX(-0.1);
        }
        else if(gamepad1.dpad_right){
            robotLocation.changeX(0.1);
        }
        else if(gamepad1.dpad_up){
            robotLocation.changeY(0.1);
        }
        else if(gamepad1.dpad_down){
            robotLocation.changeY(-0.1);
        }

        // Print out the robot's raw radians value
        telemetry.addData("Location", robotLocation);
        // Print out the robot's normalized heading in DEGREES [-180, 180]
        telemetry.addData("Heading", robotLocation.getHeading());
        // Printout robot's angle value
        telemetry.addData("Angle", robotLocation.getAngle());
    }
}
