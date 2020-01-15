package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedParkWall")
public class RedParkWall extends Localization {
    @Override
    public void runOpMode(){
        init(hardwareMap, false, Side.RED);

        waitForStart();
        updatePosition();
        sleep(250);

        moveWithEncoder(0, 65 , false);
    }
}
