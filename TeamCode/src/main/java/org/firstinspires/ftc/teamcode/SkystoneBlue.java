package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vuforia_SkyStone")
public class SkystoneBlue extends Localization {

    @Override
    public void runOpMode() {
        //init(hardwareMap, false, Side.BLUE);
        initVuforiaWebcam();
        waitForStart();
        /*updatePosition();
        moveWithEncoder(-31, 40, false);
        turnToAngle(0);
        sleep(1500);
        updatePosition();
        */
        while (!isStopRequested()) {
            if(((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()){
                telemetry.addData("I SEE THE STONE", "");
            }
            else{
                telemetry.addData("I DONT SEE OT", "");
            }

        }
    }
}
