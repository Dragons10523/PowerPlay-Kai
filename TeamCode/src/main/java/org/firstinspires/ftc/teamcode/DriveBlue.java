package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* CLASS SUMMARY:
 * Runs AbstractDrive.run() with the appropriate side set
 * contains the main runOpMode function
 * */

@TeleOp(name = "Drive Blue")
public class DriveBlue extends AbstractDrive {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        run(FieldSide.BLUE);
        zero();
    }
}
