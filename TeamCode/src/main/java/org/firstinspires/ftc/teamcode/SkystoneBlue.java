package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;


import java.util.Vector;

@TeleOp(name = "Vuforia_SkyStone")
public class SkystoneBlue extends Localization {

    double xDis;
    double count = 0;
    double counter = 0;
    double flip = -3;
    boolean done = false;
    double power = 0.65;
    @Override
    public void runOpMode() {
        {
            init(hardwareMap, false, Side.BLUE);
            initVuforiaWebcam();
            waitForStart();
            updatePosition();
            moveWithEncoder(-28, 48, false);
            sleep(500);
            updatePosition();
            turnToAngle(90);
        }//Getting in to position to start scaning of skystones
        while(opModeIsActive()){
            if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {
                VectorF matrix = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget().getTranslation();
                //matrix.get(0) = Y
                //matrix.get(1) = X
                //matrix.get(2) = Z
                {
                    while (matrix.get(1) < -5 || matrix.get(1) > 5) {
                        if (matrix.get(1) < -5) {
                            matrix = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget().getTranslation();
                            mecanums.absMove(180, 0.3, getAngle());
                            telemetry.addData("Visible Target", "true");
                            telemetry.addData("X", "%.1f", matrix.get(1));
                        }
                        if (matrix.get(1) > 5) {
                            matrix = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget().getTranslation();
                            mecanums.absMove(0, 0.3, getAngle());
                            telemetry.addData("Visible Target", "true");
                            telemetry.addData("X", "%.1f", matrix.get(1));
                        }
                        telemetry.update();
                    }
                    mecanums.stopNow();
                    sleep(500);
                }//Adjusting so the skystone is in front of the right claw
                {
                    while (cameraDis.getDistance(DistanceUnit.INCH) > 4.2) {
                        mecanums.absMove(270, 0.4, getAngle());
                        telemetry.addData("Camera Distance: ", cameraDis.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    mecanums.stopNow();
                    sleep(200);
                }// get 4 inches away from the sky stone
                {
                    leftClaw.setPosition(power);
                    sleep(1000);
                    wiggle(1000);
                    sleep(100);
                    turnToAngle(90);
                    mecanums.absMove(90, 0.5, getAngle());
                    sleep(500);
                    mecanums.stopNow();
                    updatePosition();
                    xDis = X - 10.5;
                    moveWithEncoder(-20, 55, true);
                    moveWithEncoder(20, Y, true);
                    leftClaw.setPosition(0);
                    updatePosition();
                }//go across the sky bridge and drop off block
                {
                    moveWithEncoder(-30, 65, true);
                    updatePosition();
                    moveWithEncoder(xDis, 65, true);
                    turnToAngle(90);
                }//move infront of second skystone
                {
                    while (cameraDis.getDistance(DistanceUnit.INCH) > 4.5) {
                        if(count % 10 == 0){
                            turnToAngle(90);
                        }
                        count += 1;
                        mecanums.absMove(270, 0.4, getAngle());
                        telemetry.addData("Camera Distance: ", cameraDis.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                mecanums.stopNow();
                }// get 4 inches away from the sky stone
                {
                    rightClaw.setPosition(power);
                    sleep(1000);
                    wiggle(1000);
                    sleep(100);
                    turnToAngle(90);
                    sleep(1000);
                    mecanums.absMove(90, 0.5, getAngle());
                    sleep(1000);
                    mecanums.stopNow();
                    updatePosition();
                    moveWithEncoder(20, 55, true);
                    rightClaw.setPosition(0);
                    done = true;
                }//go across the sky bridge and drop off second sky bridge

            }
            else{
                if(!done) {

                    if (counter < 8) {
                        updatePosition();
                        moveWithEncoder(X + flip, Y, false);
                        sleep(200);
                        counter++;
                        if(counter % 2 == 0){
                            turnToAngle(90);
                        }
                    } else {
                        flip = -flip;
                        counter = 0;
                    }
                }
                else{
                    break;
                }
            }
        }
        targetsSkyStone.deactivate();
    }
    void wiggle(double time){
        ElapsedTime timer = new ElapsedTime();
        long delay = 200;
        while(opModeIsActive() && timer.milliseconds() < time){
            mecanums.joystickMove(0, 0, -0.4);
            sleep(delay);
            mecanums.joystickMove(0, 0, 0.4);
            sleep(delay);
        }
        mecanums.stopNow();
    }
}