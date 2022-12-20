package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Auto", preselectTeleOp = "Drive")
public class AutoRightImpl extends Auto {
    @Override public void start() {
        auto(FieldSide.RIGHT);
    }
}
