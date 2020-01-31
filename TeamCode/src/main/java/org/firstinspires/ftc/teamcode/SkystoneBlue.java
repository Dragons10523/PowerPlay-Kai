package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;


import java.util.Vector;

@TeleOp(name = "Vuforia_SkyStone")
public class SkystoneBlue extends Localization {

    double xDis;

    @Override
    public void runOpMode() {
        {
            init(hardwareMap, false, Side.BLUE);
            initVuforiaWebcam();
            waitForStart();
            updatePosition();
            moveWithEncoder(-32, 46, false);
            sleep(500);
            updatePosition();
            turnToAngle(90, 0.4);
        }//Getting in to position to start scaning of skystones
        while(opModeIsActive()){
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                VectorF matrix = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget().getTranslation();
                //matrix.get(0) = Y
                //matrix.get(1) = X
                //matrix.get(2) = Z
                {
                    while (matrix.get(1) < -40 || matrix.get(1) > -30) {
                        if (matrix.get(1) < -40) {
                            matrix = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget().getTranslation();
                            mecanums.absMove(180, 0.3, getAngle());
                            telemetry.addData("Visible Target", "true");
                            telemetry.addData("X", "%.1f", matrix.get(1));
                        }
                        if (matrix.get(1) > -30) {
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
                    while (cameraDis.getDistance(DistanceUnit.INCH) > 4) {
                        mecanums.absMove(270, 0.4, getAngle());
                        telemetry.addData("Camera Distance: ", cameraDis.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    mecanums.stopNow();
                    sleep(200);
                }// get 4 inches away from the sky stone
                {
                    xDis = X - 14.5;
                    leftClaw.setPosition(0.85);
                    sleep(1000);
                    mecanums.absMove(90, 0.5, getAngle());
                    sleep(500);
                    mecanums.stopNow();
                    updatePosition();
                    moveWithEncoder(20, 40, true);
                    leftClaw.setPosition(0);
                    updatePosition();
                }//go across the sky bridge and drop off block
                {
                    moveWithEncoder(xDis, 49, false);
                    turnToAngle(90);
                }//move infront of second skystone
                {
                    while (cameraDis.getDistance(DistanceUnit.INCH) > 4) {
                        mecanums.absMove(270, 0.4, getAngle());
                        telemetry.addData("Camera Distance: ", cameraDis.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                mecanums.stopNow();
                }// get 4 inches away from the sky stone
                {
                    rightClaw.setPosition(0.85);
                    sleep(1000);
                    mecanums.absMove(90, 0.5, getAngle());
                    sleep(1000);
                    mecanums.stopNow();
                    updatePosition();
                    moveWithEncoder(20, 55, true);
                    rightClaw.setPosition(0);
                }//go across the sky bridge and drop off second sky bridge

            }
            else{
                moveWithEncoder(X - 4, Y, false);
                sleep(250);
                updatePosition();
            }
        }
        targetsSkyStone.deactivate();
    }
}