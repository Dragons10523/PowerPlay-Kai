package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.processors.AutoControl;
import org.firstinspires.ftc.teamcode.processors.SignalOpticalSystem;

import java.util.Arrays;

@Autonomous(name = "SignalTest", group = "Test")
public class SignalTest extends AutoControl {
    @Override
    public void loop() {
        if(isStopRequested) return;

        SignalOpticalSystem.SignalOrientation signalOrientation = signalOpticalSystem.getSignalOrientation();
        telemetry.addLine(signalOrientation.name());
        telemetry.addData("Center", Arrays.toString(signalOpticalSystem.centerLab));
        telemetry.update();
    }
}
