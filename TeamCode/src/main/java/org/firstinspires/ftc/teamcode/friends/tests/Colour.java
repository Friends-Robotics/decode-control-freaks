package org.firstinspires.ftc.teamcode.friends.tests;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;

public class Colour {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(6);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;

        telemetry.addData("Current Red: ", normRed);
        telemetry.addData("Current Green: ", normGreen);
        telemetry.addData("Current Blue: ", normBlue);

        /*

        /// PURPLE
        RED = 0.45 - 1
        GREEN = 0 - 0.5
        BLUE = 0 - 1

        /// GREEN
        RED =
        GREEN =
        BLUE =

         */

        return DetectedColor.UNKNOWN;
    }
}