package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueFoundation")
public class AutnomousBlue extends Localization {
    @Override
    public void runOpMode(){
        initialize(hardwareMap);
        waitForStart();

        funcs.absMove(270, 0.45, getAngle());
        sleep(500);
        funcs.stopNow();

        updatePosition();
        telemetry.addData("X Y", X + " " + Y);
        telemetry.update();
        sleep(5000);
        moveWithEncoder(50, 35);
        if(getAngle() < 0-0.2){
            while(getAngle()<0-0.2){
                funcs.absMove(0,0.20,0.25);
            }
            funcs.stopNow();
        }
        rightClaw.setPosition(0.75);
        leftClaw.setPosition(0.75);
        sleep(1500);
        updatePosition();
        moveWithEncoder(X, 45);
        sleep(2000);
        turnToAngle(90, true);
    }
}
