package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor")
public class ColorDistance extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorSensor color = hardwareMap.get(ColorSensor.class, "cd");
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "cd");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Color", color.alpha());
            telemetry.update();
        }
    }
}
