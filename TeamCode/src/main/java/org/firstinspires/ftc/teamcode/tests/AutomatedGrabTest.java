package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control;

@TeleOp(name = "GrabTest", group = "Test")
public class AutomatedGrabTest extends Control {
    @Override
    public void start() {
        super.start();
        armControl.shouldRun = true;
    }

    @Override
    public void loop() {
        super.loop();

        boolean clawOpen = armControl.isClawOpen();
        if(clawOpen && gamepad1.x && clawDistance() <= 1.5) {
            clawOpen = false;
        }

        if(clawOpen != armControl.isClawOpen()) {
            armControl.toggleClaw();
        }
    }
}
