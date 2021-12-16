package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Duck", group = "Auto Main", preselectTeleOp = "Drive Red")
public class RedDuck extends AbstractDuck {
    @Override
    public void runOpMode() throws InterruptedException {
        run(FieldSide.RED);
        zero();
    }
}
