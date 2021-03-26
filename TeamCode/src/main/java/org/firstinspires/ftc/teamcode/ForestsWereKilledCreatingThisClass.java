package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "PaperCausesManyPeoplePainNightmareNightmareNightmareNightmareNightmareNightmare")
public class ForestsWereKilledCreatingThisClass extends Localization{

    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        startLocalization();
        vwompClamp(false);
        waitForStart();
        final int r = rings();
        sleep(3000);
        stopTfodCrap();
//        final int r = 0;
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()){
                    telemetry.addData("Rings", r);
                    telemetry.addData("theta", theta * 180 / Math.PI);
                    telemetry.addData("F", thalatte.front.getDistance(DistanceUnit.INCH));
                    telemetry.addData("B", thalatte.back.getDistance(DistanceUnit.INCH));
                    telemetry.addData("R", thalatte.right.getDistance(DistanceUnit.INCH));
                    telemetry.addData("L", thalatte.left.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                    sleep(30);
                }
            }
        }).start();
        drive(0.7,0.7);
        sleep(500);
        drive(-1,-1);
        sleep(200);
        drive(0.7,0.7);
        while(thalatte.back.getDistance(DistanceUnit.INCH) < 5);
        drive(0,0);
        shoot(1);
        sleep(500);
        intake(true);
        feeder(true);
        sleep(3000);
        intake(false);
        feeder(false);
        shoot(0);
        switch(r){
            case 4:
                drive(0.7,0.7);
                while(thalatte.front.getDistance(DistanceUnit.INCH) > 30);
                drive(0,0);
                startTurnTo(Math.PI + 0.5);
                while(turningFlag) updateLocalization();
                vwompArm(1);
                sleep(1300);
                vwompArm(0);
                vwompClamp(true);
                sleep(300);
                vwompArm(-1);
                sleep(1300);
                vwompArm(0);
                startTurnTo(Math.PI / 2);
                while(turningFlag) updateLocalization();
                drive(-0.7,-0.7);
                while(thalatte.front.getDistance(DistanceUnit.INCH) < 28);
                drive(0,0);
                break;
            case 1:
                drive(0.7,0.7);
                while(thalatte.front.getDistance(DistanceUnit.INCH) > 56 );
                drive(0,0);
                startTurnTo(0.5);
                while(turningFlag) updateLocalization();
                vwompArm(1);
                sleep(1300);
                vwompArm(0);
                vwompClamp(true);
                sleep(300);
                vwompArm(-1);
                sleep(1300);
                vwompArm(0);
                startTurnTo(Math.PI / 2);
                while(turningFlag) updateLocalization();
                drive(-0.5,-0.5);
                while(thalatte.front.getDistance(DistanceUnit.INCH) < 42);
                drive(0,0);
                break;
            case 0:
                drive(0.6,0.6);
                ElapsedTime e = new ElapsedTime();
                while(thalatte.back.getDistance(DistanceUnit.INCH) < 67 && e.seconds() < 1.3);
                sleep(200);
                drive(0,0);
                startTurnTo(Math.PI + 0.5);
                while(turningFlag) updateLocalization();
                vwompArm(1);
                sleep(1300);
                vwompArm(0);
                vwompClamp(true);
                sleep(300);
                vwompArm(-1);
                sleep(1300);
                vwompArm(0);
                break;
        }
        zero();
    }
}
