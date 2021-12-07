package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive")
public class Drive extends Localization {
    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        startLocalization(vuFlag.OFF);
        waitForStart();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()) {
                    telemetry.addData("Theta"           , theta * 180 / Math.PI                                        );
                    telemetry.addData("Shooter"         , thalatte .shooterFront.getPower()    != 0 ? "ON"   : "OFF"   );
                    telemetry.addData("Shooter Power"   , sPower                                                       );
                    telemetry.addData("Intake"          , thalatte .intake      .getPower()    != 0 ? "ON"   : "OFF"   );
                    telemetry.addData("Reversed",       Math       .signum(FEEDER)             >  0 ? "Yes"  : "No"    );
                    telemetry.addData("Feeder"          , thalatte .feeder      .getPower()    != 0 ? "ON"   : "OFF"   );
                    telemetry.addData("Clamp"           , thalatte .vwompClamp  .getPosition() != 0 ? "OPEN" : "CLOSED");
//                    telemetry.addData("Aiming"          , aimingFlag                                                   );
//                    telemetry.addData("m10",cv.m10);
//                    telemetry.addData("m00",cv.m00);
//                    telemetry.addData("cols",cv.cols);
//                    telemetry.addData("averagePosition", cv.averagePosition());
//                    telemetry.addData("IsInited", isInited);
//                    telemetry.addData("IsVisible", targetVisible);
////                    if(isInited) {
////                        updateLocalization();
////                        telemetry.addData("Trans Length", translation.length());
////                        telemetry.addData("Translation[0]", translation.get(0));
////                        telemetry.addData("Translation[1]", translation.get(1));
////                        telemetry.addData("Translation[2]", translation.get(2));
////                        telemetry.addData("Rotation Ex", rotation.thirdAngle);
////                        telemetry.addData("Rotation In", rotation2.thirdAngle);
////                    }
                    telemetry.update();
                    sleep(50);
                }
            }
        }).start();

        while(opModeIsActive()) {
            GamepadPrev.ButtonEvent ev1 = GamepadPrev.ButtonEvent.NONE;
            GamepadPrev.ButtonEvent ev2 = GamepadPrev.ButtonEvent.NONE;

            if(turningFlag /*|| aimingFlag*/) updateLocalization();
            while((ev1 = prev1.getEvent()) != GamepadPrev.ButtonEvent.NONE || (ev2 = prev2.getEvent()) != GamepadPrev.ButtonEvent.NONE){
                switch(ev1){
//                    case A:
//                        autoAim();
//                        break;
//                    case B:
//                        turningFlag = false;
//                        aimingFlag  = false;
//                        break;
                    case LEFT:
                        if(!turningFlag) startTurnTo(Math.PI / 2);
                        break;
                    case RIGHT:
                        if(!turningFlag) startTurnTo(3 * Math.PI / 2);
                        break;
                    case UP:
                        if(!turningFlag) startTurnTo(0);
                        break;
                    case DOWN:
                        if(!turningFlag) startTurnTo(Math.PI);
                }
                switch(ev2){
                    case A:
                        toggleFeeder();
                        break;
                    case B:
                        toggleIntake();
                        break;
                    case RB:
                        toggleVwompClamp();
                        break;
                    case X:
                    case Y:
                        toggleShoot(sPower);
                        break;
                    case LEFT:
                    case RIGHT:
                        toggleDirection();
                        break;
                    case UP:
                        sPower = Math.min(sPower + 0.1, 1);
                        break;
                    case DOWN:
                        sPower = Math.max(sPower - 0.1, 0);
                        break;
                }
            }
                 if(gamepad1.left_trigger > 0 ) setSpeed(Speed.FAST);
            else if(gamepad1.left_bumper      ) setSpeed(Speed.NORMAL);
            else if(gamepad1.right_bumper     ) setSpeed(Speed.SLOW  );
            else if(gamepad1.right_trigger > 0) setSpeed(Speed.SLOWER);
            else                                setSpeed(Speed.FASTER);

            vwompArm(-gamepad2.left_stick_y);

            if(!(turningFlag /*|| aimingFlag*/)) driveLoop();
        }
        zero();
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭