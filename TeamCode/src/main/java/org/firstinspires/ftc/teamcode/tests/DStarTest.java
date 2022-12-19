package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;

@Autonomous(name = "DStarTest", group = "Test")
public class DStarTest extends AutoControl {
    @Override
    public void loop() {
        if(isStopRequested) return;
        super.loop();

        kai.deadwheels.setTransform(1, 1, 0);

        moveToTile(1, 3);

        requestOpModeStop();
    }
}
