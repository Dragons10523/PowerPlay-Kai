package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BluePark")
public class BluePark extends Localization{
    @Override
    public void runOpMode(){
        initialize(hardwareMap);
        waitForStart();
        updatePosition();
        moveWithEncoder(0, 65);
    }
}
