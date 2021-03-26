package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive")
public class Drive extends Control {
    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        waitForStart();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()) {
//                    telemetry.addData("Theta", (thalatte.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle) * 180 / Math.PI);
                    telemetry.addData("Shooter"         , thalatte.shooterFront.getPower()    != 0 ? "ON"   : "OFF"   );
                    telemetry.addData("Shooter Power"   , sPower                                                      );
                    telemetry.addData("Intake"          , thalatte.intake      .getPower()    != 0 ? "ON"   : "OFF"   );
                    telemetry.addData("Intake Direction", Math.signum(INTAKE)                 == 1 ? "1"    : "-1"    );
                    telemetry.addData("Feeder Direction", Math.signum(FEEDER)                 == 1 ? "1"    : "-1"    );
                    telemetry.addData("Feeder"          , thalatte.feeder      .getPower()    != 0 ? "ON"   : "OFF"   );
                    telemetry.addData("Clamp"           , thalatte.vwompClamp  .getPosition() != 0 ? "OPEN" : "CLOSED");
                    telemetry.update();
                    sleep(50);
                }
            }
        }).start();

        while(opModeIsActive()) {
            GamepadPrev.ButtonEvent ev1 = GamepadPrev.ButtonEvent.NONE;
            GamepadPrev.ButtonEvent ev2 = GamepadPrev.ButtonEvent.NONE;

            while((ev1 = prev1.getEvent()) != GamepadPrev.ButtonEvent.NONE || (ev2 = prev2.getEvent()) != GamepadPrev.ButtonEvent.NONE){
//                switch(ev1){
//
//                }
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
                        toggleFeederDirection();
                        break;
                    case RIGHT:
                        toggleIntakeDirection();
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

            driveLoop();
        }
        zero();
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭