package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive")
public class Drive extends Localization {
    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        waitForStart();

        while(opModeIsActive()) {
            GamepadPrev.ButtonEvent ev1 = GamepadPrev.ButtonEvent.NONE;
            GamepadPrev.ButtonEvent ev2 = GamepadPrev.ButtonEvent.NONE;
            while((ev1 = prev1.getEvent()) != GamepadPrev.ButtonEvent.NONE || (ev2 = prev2.getEvent()) != GamepadPrev.ButtonEvent.NONE){
//                switch(ev1){
//
//                }
                switch(ev2){
                    case LB:
                        powerShotCycle(-1);
                        break;
                    case RB:
                        powerShotCycle(1);
                        break;
                }
            }
                 if(gamepad1.left_trigger > 0)  setSpeed(Speed.FASTER);
            else if(gamepad1.left_bumper)       setSpeed(Speed.FAST);
            else if(gamepad1.right_bumper)      setSpeed(Speed.SLOW);
            else if(gamepad1.right_trigger > 0) setSpeed(Speed.SLOWER);
            else setSpeed(Speed.NORMAL);
            driveLoop();
        }
        zero();
    }
}
