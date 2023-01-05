package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Auto", preselectTeleOp = "Drive")
public class AutoLeftImpl extends Auto {
    @Override public void start() {
        auto(FieldSide.LEFT);
    }
}