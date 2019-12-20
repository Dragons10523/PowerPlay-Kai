package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BlueFoundation")
public class AutnomousBlue extends Localization {
    @Override
    public void runOpMode(){
        initialize(hardwareMap);
        if(left.getDistance(DistanceUnit.INCH) < 5 || right.getDistance(DistanceUnit.INCH) < 5){
            telemetry.addData("HEY", "Fix the F'in arms");
        }
        telemetry.update();
        waitForStart();

        funcs.absMove(270, 0.45, getAngle());
        sleep(500);
        funcs.stopNow();

        updatePosition();
        telemetry.addData("X Y", X + " " + Y);
        telemetry.update();
        sleep(1000);
        moveWithEncoder(50, 35);

        boolean leftLimit = !leftLimitSwitch.getState();

        while(!leftLimit && opModeIsActive()){
            funcs.joystickMove(0,0, -0.1);
            leftLimit = !leftLimitSwitch.getState();
        }
        funcs.stopNow();

        rightClaw.setPosition(0.75);
        leftClaw.setPosition(0.75);
        sleep(1500);
        updatePosition();
        moveWithEncoder(X, 45);
        sleep(1000);
        turnToAngle(90, true, 0.4);

        sleep(500);

       rightClaw.setPosition(0);
       leftClaw.setPosition(0);

        sleep(500);
        turnToAngle(0, false);
        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.5);
        sleep(750);
        updatePosition();
        sleep(200);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        telemetry.addData("X Y", X + "   " + Y);
        telemetry.update();

        moveWithEncoder(0, 39); //Closer to the middle
        //moveWithEncoder(3, 64); //against the wall

        if(getAngle() > 1){
            choice = false; //turn right
        }
        else if(getAngle() < -1){
            choice = true; //turn left
        }
        else
        turnToAngle(0, choice);
        while(opModeIsActive());

    }
}
