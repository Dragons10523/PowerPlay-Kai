package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractBarcode;

@Autonomous(name = "OpenCVTesting", group = "Testing")
public class OpenCVColorCalibrate extends AbstractBarcode {

    @Override
    public void runOpMode() throws InterruptedException {
        startOpenCV();

        waitForStart();

        while(opModeIsActive()) {
            double[] centerColor = hueTrackingPipeline.getCenterColorLab();
            double[] centerColorVanilla = hueTrackingPipeline.getCenterColorVanilla();
            telemetry.addData("Center Vanilla", centerColorVanilla[0] + " " + centerColorVanilla[1] + " " + centerColorVanilla[2]);
            telemetry.addData("Center L*A*B*", (100*centerColor[0]/255) + " " + (centerColor[1]-128) + " " + (centerColor[2]-128));
            telemetry.addData("Tracking Pipeline Overhead", ahi.camera.getTotalFrameTimeMs());
            telemetry.update();
            sleep(20);
        }

        stopOpenCV();
    }
}
