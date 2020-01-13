package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedParkWall")
public class RedParkWall extends NewLocalization {
    @Override
    public void runOpMode(){
        init(hardwareMap, false, Side.RED);

        waitForStart();
        updatePosition(1);
        sleep(250);

        moveWithEncoder(0, 65 , false);
    }
}
