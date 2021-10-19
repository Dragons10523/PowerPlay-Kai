package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control;

@Autonomous(name = "ArmControlTest", group = "Testing")
public class ArmControlTest extends Control {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        armControl(ArmPosition.PICKUP);
        if(protectedSleep(1000)) return;

        armControl(ArmPosition.HIGH);
        if(protectedSleep(1000)) return;

        armControl(ArmPosition.MED);
        if(protectedSleep(1000)) return;

        armControl(ArmPosition.LOW);
        if(protectedSleep(1000)) return;

        zero();
    }
}
