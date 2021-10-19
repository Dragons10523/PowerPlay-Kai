package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Freight", preselectTeleOp = "Drive Blue")
public class BlueFreight extends AbstractFreight {
    @Override
    public void runOpMode() throws InterruptedException {
        run(FieldSide.BLUE);
        zero();
    }
}
