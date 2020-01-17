package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vuforia_SkyStone")
public class SkystoneBlue extends Localization {
    @Override
    public void runOpMode() {
        init(hardwareMap, false, Side.BLUE);
        initVuforia();
        waitForStart();
        updatePosition();
        moveWithEncoder(-30, 40, false);
        turnToAngle(0);
        sleep(200);
        updatePosition();
        if(!((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()){
            moveWithEncoder(-38, 40, false);
        }
    }
}
