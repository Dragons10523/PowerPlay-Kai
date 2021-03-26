package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Test")
public class  Test extends Localization {
    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        startLocalization(false);
        waitForStart();
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()){
                    telemetry.addData("theta", theta * 180 / Math.PI);
                    telemetry.update();
                    sleep(30);
                }
            }
        }).start();
        sleep(500);
        startTurnTo(Math.PI);
        while(turningFlag) updateLocalization();
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭