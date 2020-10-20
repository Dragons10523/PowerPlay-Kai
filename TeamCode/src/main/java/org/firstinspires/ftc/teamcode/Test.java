package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "ShootyShoot")
public class Test extends Control {

    double power = 0;
    short negator = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        waitForStart();
        while (opModeIsActive()) {
            GamepadPrev.ButtonEvent ev1;

            do {
                ev1 = prev1.getEvent();
                switch(ev1) {
                    case UP:
                        power = 1;
                        break;
                    case DOWN:
                        power = 0;
                        break;
                    case LEFT:
                        power = clamp(power-0.2,0.0,1.0);
                        break;
                    case RIGHT:
                        power = clamp(power+0.2,0.0,1.0);
                        break;
                    case X:
                        negator = (short)(-negator);
                        break;
                }
            } while(ev1 != GamepadPrev.ButtonEvent.NONE);

            shoot(negator * power);
            telemetry.addData("Shooting Power", negator * power);
            telemetry.update();
        }
    }
}
