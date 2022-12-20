package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Auto")
public class AutoRightImpl extends Auto {
    @Override public void start() {
        auto(FieldSide.RIGHT);
    }
}
