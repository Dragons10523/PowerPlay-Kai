package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Freight", group = "Main", preselectTeleOp = "Drive Red")
public class RedFreight extends AbstractFreight {
    @Override
    public void runOpMode() throws InterruptedException {
        run(FieldSide.RED);
        zero();
    }
}
