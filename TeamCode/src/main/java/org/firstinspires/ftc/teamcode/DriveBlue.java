package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive Blue")
public class DriveBlue extends AbstractDrive {

    @Override
    public void runOpMode() throws InterruptedException {
        run(FieldSide.BLUE);
    }
}
