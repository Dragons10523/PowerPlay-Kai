package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control;

@Autonomous(name = "DDRTest", group = "Testing")
public class DDRTest extends Control {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        telemetry.addLine("Testing DDR Forwards");
        telemetry.update();
        playDDR(1);
        if(protectedSleep(1000)) return;
        playDDR(0);
        if(protectedSleep(500)) return;

        telemetry.addLine("Testing DDR Backwards");
        telemetry.update();
        playDDR(-1);
        if(protectedSleep(1000)) return;
        playDDR(0);
        if(protectedSleep(500)) return;

        zero();
    }
}
