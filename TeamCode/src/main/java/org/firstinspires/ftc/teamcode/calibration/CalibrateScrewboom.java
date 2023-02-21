package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.processors.Control;

@TeleOp(name = "Calibrate Screwboom", group = "Calibration")
public class CalibrateScrewboom extends Control {
    @Override
    public void loop() {
        super.loop();

        kai.deadwheels.setTransform(1.5, -(3.5/24), 0);
        armControl.setTarget(48, 24);
    }
}
