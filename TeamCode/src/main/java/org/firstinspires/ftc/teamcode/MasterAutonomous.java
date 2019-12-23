package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MasterAuto")
@Disabled
public class MasterAutonomous extends Localization {
    @Override
    public void runOpMode(){
        initialize(hardwareMap);

        while(left.getDistance(DistanceUnit.INCH) < 5 || right.getDistance(DistanceUnit.INCH) < 5 && opModeIsActive()){
            telemetry.addData("HEY", "Fix the F'in arms");
            telemetry.update();
        }

        waitForStart();

        updatePosition();

        if(corner == Corner.BlueTri){
            color = Color.BLUE;
        }
        else{
            color = Color.RED;
        }

        if(color == Color.BLUE){
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
            turnToAngle(180, true, 0.4);

            sleep(500);

            rightClaw.setPosition(0);
            leftClaw.setPosition(0);

            sleep(500);
            turnToAngle(90, false);
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
                turnToAngle(90, choice);
        }

        else{
            funcs.absMove(270, 0.45, getAngle());
            sleep(500);
            funcs.stopNow();

            updatePosition();
            telemetry.addData("X Y", X + " " + Y);
            telemetry.update();
            sleep(1000);
            moveWithEncoder(50, -35);

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
            moveWithEncoder(X, -45);
            sleep(1000);
            turnToAngle(0, true, 0.4);

            sleep(500);

            rightClaw.setPosition(0);
            leftClaw.setPosition(0);

            sleep(500);
            turnToAngle(90, false);
            rightClaw.setPosition(0.5);
            leftClaw.setPosition(0.5);
            sleep(750);
            updatePosition();
            sleep(200);
            rightClaw.setPosition(0);
            leftClaw.setPosition(0);
            telemetry.addData("X Y", X + "   " + Y);
            telemetry.update();

            //moveWithEncoder(0, -39); //Closer to the middle
            moveWithEncoder(3, -64); //against the wall

            if(getAngle() > 1){
                choice = false; //turn right
            }
            else if(getAngle() < -1){
                choice = true; //turn left
            }
            else
                turnToAngle(90, choice);
        }

    }
}
