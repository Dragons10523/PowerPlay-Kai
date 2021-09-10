package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Basic Auto Shooting")
public class BasicAutoShooting extends Localization{

    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        startLocalization(vuFlag.AUTO);
        vwompClamp(false);
        stopTfodCrap();
        waitForStart();

        drive(0.7,0.7);
        sleep(500);
        drive(-1,-1);
        sleep(200);

        drive(0.7,0.7);
        while(thalatte.back.getDistance(DistanceUnit.INCH) < 5);
        drive(0,0);

        startTurnTo((Math.PI/2) - 0.1);
        while(turningFlag) updateLocalization();
        shoot(.98);
        sleep(2500);
        intake(true);
        feeder(true);
        sleep(3000);
        intake(false);
        feeder(false);
        shoot(0);

        startTurnTo(Math.PI/2);
        while(turningFlag) updateLocalization();
        sleep(500);

        driveDist(48.0, 0.5);
        /*drive(0.6, 0.6);
        ElapsedTime e = new ElapsedTime();
        while(thalatte.back.getDistance(DistanceUnit.INCH) < 67 && e.seconds() < 1.3);
        sleep(200);*/
    }
}
