package org.firstinspires.ftc.teamcode.Auto;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


public class ColorSensorClass {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    float[] hsvValues = {0F, 0F, 0F};
    final float[] values = hsvValues;
    final double SCALE_FACTOR = 225;
    public ColorSensorClass(ColorSensor sensorColor){
        this.sensorColor = sensorColor;
    }

    //must be Called after opMode is started
    void updateColorSensor() {
        Color.RGBToHSV(
                (int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues
        );

//        telemetry.addData("Distance (cm)",
//                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//        telemetry.addData("Alpha", sensorColor.alpha());
//        telemetry.addData("Red  ", sensorColor.red());
//        telemetry.addData("Green", sensorColor.green());
//        telemetry.addData("Blue ", sensorColor.blue());
//        telemetry.addData("Hue", hsvValues[0]);
//        telemetry.update();
    }
    public float[] getHsvValues(){
        updateColorSensor();
        return hsvValues;
    }

}
