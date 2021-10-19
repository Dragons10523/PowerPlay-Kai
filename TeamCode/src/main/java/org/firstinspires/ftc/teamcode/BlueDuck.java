package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Duck", preselectTeleOp = "Drive Blue")
public class BlueDuck extends AbstractDuck {
    @Override
    public void runOpMode() throws InterruptedException {
        run(FieldSide.BLUE);
    }
}