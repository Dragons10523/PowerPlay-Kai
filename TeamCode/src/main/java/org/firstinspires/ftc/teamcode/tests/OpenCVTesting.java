package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractBarcode;

@Autonomous(name = "OpenCVTesting", group = "Testing")
public class OpenCVTesting extends AbstractBarcode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeValues();
        startOpenCV();

        waitForStart();

        while(!hueTrackingPipeline.isPipelineReady() && opModeIsActive()) sleep(50);

        hueTrackingPipeline.startVideo();

        while(opModeIsActive()) {
            double[] centerColor = hueTrackingPipeline.getCenterColorLab();
            telemetry.addData("Center L*A*B*", (100*centerColor[0]/255) + " " + (centerColor[1]-128) + " " + (centerColor[2]-128));
            telemetry.addData("Pipeline Time", ahi.camera.getPipelineTimeMs());
            telemetry.addData("Total Time", ahi.camera.getTotalFrameTimeMs());
            telemetry.addData("FPS", ahi.camera.getFps());
            telemetry.update();

            sleep(20);
        }

        stopOpenCV();
    }
}
