package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueFoundation")
public class AutnomousBlue extends Localization {
    @Override
    public void runOpMode(){
        initialize();
        waitForStart();

        updatePosition();
        telemetry.addData("X Y", X + " " + Y);
        telemetry.update();
        sleep(5000);
        moveWithEncoder(50, 24);
        rightClaw.setPosition(0.75);
        leftClaw.setPosition(0.75);
        sleep(2000);
    }
}
