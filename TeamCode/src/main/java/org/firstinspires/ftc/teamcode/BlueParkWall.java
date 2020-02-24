package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueParkWall")
public class BlueParkWall extends Localization {
    @Override
    public void runOpMode(){
        init(hardwareMap, false, Side.BLUE);

        waitForStart();
        sleep(3000);
        updatePosition();
        sleep(250);

        moveWithEncoder(0, 65 , false);
    }
}
