package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.processors.Control;

@TeleOp(name = "DriftCalculator", group = "Test")
public class DriftCalculator extends Control {
    int selectedPole = 12;
    boolean directionPrev = false;

    @Override
    public void start() {
        super.start();
        kai.liftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("Velocity: ", kai.liftExtension.getCurrentPosition()/getRuntime());
    }
}
