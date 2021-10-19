package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* CLASS SUMMARY:
 * Runs AbstractDrive.run() with the appropriate side set
 * contains the main runOpMode function
 * */

@TeleOp(name = "Drive Red")
public class DriveRed extends AbstractDrive {

    @Override
    public void runOpMode() throws InterruptedException {
        run(FieldSide.RED);
    }
}
