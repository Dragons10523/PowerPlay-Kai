package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
@TeleOp(name = "VectorF_Skystone")
public class TestVectorF extends Localization {
    @Override
    public void runOpMode() {
        init(hardwareMap, false, Side.BLUE);
        initVuforiaWebcam();
        waitForStart();
        while (opModeIsActive()) {
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                VectorF matrix = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget().getTranslation();
                //matrix.get(0) = Y
                //matrix.get(1) = X
                //matrix.get(2) = Z
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
                while(robot.cameraDis.getDistance(DistanceUnit.INCH) > 3){
                    mecanums.absMove(270,0.5, getAngle());
                }
            }
        }
    }
}
