package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.SignalOpticalSystem;

@Autonomous(name = "SignalTest", group = "Test")
public class SignalTest extends AutoControl {
    @Override
    public void loop() {
        if(isStopRequested) return;
        super.loop();

        SignalOpticalSystem.SignalOrientation signalOrientation = signalOpticalSystem.getSignalOrientation();
        telemetry.addLine(signalOrientation.name());
        telemetry.update();
        isStopRequested = true;
    }
}
