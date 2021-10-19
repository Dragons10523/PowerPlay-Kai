package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control;

@Autonomous(name = "ToolControlTest", group = "Testing")
public class ToolControlTest extends Control {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        telemetry.addLine("Testing Intake");
        telemetry.update();
        runIntake(true);
        if(protectedSleep(1000)) return;
        runIntake(false);
        if(protectedSleep(500)) return;

        telemetry.addLine("Testing DDR");
        telemetry.update();
        playDDR(1);
        if(protectedSleep(1000)) return;
        playDDR(-1);
        if(protectedSleep(1000)) return;
        playDDR(0);
        if(protectedSleep(500)) return;

        telemetry.addLine("Testing Flup");
        telemetry.update();
        setFlup(true);
        if(protectedSleep(1000)) return;
        setFlup(false);
        if(protectedSleep(500)) return;

        telemetry.addLine("Testing Lift");
        telemetry.update();
        setLiftPower(0.4);
        if(protectedSleep(1000)) return;
        setLiftPower(-0.4);
        if(protectedSleep(1000)) return;
        setLiftPower(0.0);
        if(protectedSleep(500)) return;

        zero();
    }
}
