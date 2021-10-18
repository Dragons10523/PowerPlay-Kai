package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive Red")
public class DriveRed extends AbstractDrive {

    @Override
    public void runOpMode() throws InterruptedException {
        run(FieldSide.RED);
    }
}
