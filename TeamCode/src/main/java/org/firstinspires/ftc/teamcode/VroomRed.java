package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* CLASS SUMMARY:
 * Runs AbstractDrive.run() with the appropriate side set
 * contains the main runOpMode function
 * */

@TeleOp(name = "Vroom Vroom Red")
public class VroomRed extends AbstractVroom {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        run(FieldSide.RED);
        zero();
    }
}
