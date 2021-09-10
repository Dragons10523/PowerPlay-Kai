package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Basic Auto")
public class BasicAuto extends Localization{

    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        startLocalization(vuFlag.AUTO);
        vwompClamp(false);
        waitForStart();

        stopTfodCrap();

        driveDist(48.0, 0.5);
        /*drive(0.6, 0.6);
        ElapsedTime e = new ElapsedTime();
        while(thalatte.back.getDistance(DistanceUnit.INCH) < 67 && e.seconds() < 1.3);
        sleep(200);*/
    }
}
