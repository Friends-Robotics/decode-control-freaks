package org.firstinspires.ftc.teamcode.HarryTestAndNotes.Hardware.Mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBoard2 {
    private DigitalChannel touchSensor;
    // Normally want to name it with what the sensor does like armInPositionTouchSensor

    public void init(HardwareMap hwMap){
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isTouchSensorPressed(){
        return (!touchSensor.getState());
        // When pressed it now correctly returns true
    }
    public boolean isTouchSensorReleased(){
        return touchSensor.getState();
    }

    public String toString(boolean bool){
        if(bool){
            return "true";
        }
        else if(!bool){
            return "false";
        }
        return null;
    }

    public String isTouchSensorPressedText(){
        String isPressed = toString(isTouchSensorPressed());
        if(isPressed.equals("true")){
            return "Pressed";
        }
        else if(isPressed.equals("false")){
            return "Not Pressed";
        }
        return "";
    }
}
