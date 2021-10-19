package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control;

@Autonomous(name = "ArmControlTest", group = "Testing")
public class ArmControlTest extends Control {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        telemetry.addLine("Moving arm to PICKUP");
        telemetry.update();
        armControl(ArmPosition.PICKUP);
        if(protectedSleep(1000)) return;

        telemetry.addLine("Moving arm to HIGH");
        telemetry.update();
        armControl(ArmPosition.HIGH);
        if(protectedSleep(1000)) return;

        telemetry.addLine("Moving arm to MED");
        telemetry.update();
        armControl(ArmPosition.MED);
        if(protectedSleep(1000)) return;

        telemetry.addLine("Moving arm to LOW");
        telemetry.update();
        armControl(ArmPosition.LOW);
        if(protectedSleep(1000)) return;

        zero();
    }
}
