package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Freight", group = "Auto Main", preselectTeleOp = "Vroom Vroom Blue")
public class BlueFreight extends AbstractFreight {
    @Override
    public void runOpMode() throws InterruptedException {
        run(FieldSide.BLUE);
        zero();
    }
}
