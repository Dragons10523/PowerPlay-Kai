package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive")
public class Drive extends Localization {

    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        waitForStart();
        while(opModeIsActive()) {

            drive(-gamepad1.left_stick_y,-gamepad1.right_stick_y);

            shoot(Math.abs(gamepad2.left_stick_y));

            vwompArm(-gamepad2.right_stick_y);

            if(gamepad2.b)
                toggleIntake();

            if(gamepad2.x)
                toggleVwompClamp();

        }
    }
}
