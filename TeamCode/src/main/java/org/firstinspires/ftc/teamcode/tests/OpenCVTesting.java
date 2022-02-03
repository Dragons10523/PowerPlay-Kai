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
        //hueTrackingPipeline.startVideo();

        hueTrackingPipeline.setRectProc(true);
        hueTrackingPipeline.setSetpointLab(parseCalibrationFile("blu"));

        while(opModeIsActive()) {
            double[] centerColor = hueTrackingPipeline.getCenterColorLab();
            telemetry.addData("Pipeline Time", ahi.camera.getPipelineTimeMs());
            telemetry.addData("Total Time", ahi.camera.getTotalFrameTimeMs());
            telemetry.addData("Height", hueTrackingPipeline.getLargestRect().height);
            telemetry.addData("FPS", ahi.camera.getFps());
            telemetry.update();

            sleep(20);
        }

        stopOpenCV();
    }
}
