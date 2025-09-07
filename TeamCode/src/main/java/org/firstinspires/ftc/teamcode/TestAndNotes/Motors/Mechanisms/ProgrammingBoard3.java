package org.firstinspires.ftc.teamcode.TestAndNotes.Motors.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBoard3{

    private DigitalChannel touchSensor;
    private DcMotor motor; // Normally you want to name the motor with what it does

    public void init(HardwareMap hwMap){
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        motor = hwMap.get(DcMotor.class, "motor");
        // This assigns to the variable motor to the hardware that is in the configuration file of type
        // DcMotor.class and with the name of motor.
        // This name has to match exactly what is in config file

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // This sets how we want to use the motor
        // It used the encoder on the motor so that we are setting a speed and it figures out how to
        // modify power to get to that speed.

        // We like this because if we set two motors to the same speed then they have a better chance
        // at being the same speed than any other mode
    }
    public boolean isTouchSensorPressed(){
        return !touchSensor.getState();
    }
    public void setMotorSpeed(double speed){
        motor.setPower(speed);
    }

}