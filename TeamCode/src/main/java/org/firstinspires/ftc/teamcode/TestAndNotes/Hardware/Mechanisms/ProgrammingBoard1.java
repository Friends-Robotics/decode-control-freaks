package org.firstinspires.ftc.teamcode.TestAndNotes.Hardware.Mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

//One of the huge advantages of splitting things out is that we can isolate hardware "weirdness".
//For example, pushing in the touch sensor returns false and it not pushed in was true

// LOOK AT PROGRAMMING BOARD 2

public class ProgrammingBoard1 {
    private DigitalChannel touchSensor;
    // Normally want to name it with what the sensor does like armInPositionTouchSensor

    public void init(HardwareMap hwMap){// Hardware map is how the programs get info from config file of robot

        // This assigns touch sensor to the hardware that is in the config file of type DigitalChannel.class
        // Name has to match exactly what is in the config file
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");

        // You can set the DigitalChannel as either INPUT or OUTPUT
        // Since we are reading from the sensor we need output
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getTouchSensorState(){
        return touchSensor.getState();
        // This class method is used so we can read the state of the touch sensor
        // Outside of the class as it is set to private
    }
}
