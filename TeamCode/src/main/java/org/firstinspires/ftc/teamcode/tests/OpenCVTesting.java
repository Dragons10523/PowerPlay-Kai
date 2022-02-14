package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractBarcode;

@Autonomous(name = "OpenCVTesting", group = "Testing")
public class OpenCVTesting extends AbstractBarcode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeValues();
        startOpenCV();

        hueTrackingPipeline.setRectProc(true);
        hueTrackingPipeline.setSetpointLab(parseCalibrationFile("blu"));

        waitForStart();
        //hueTrackingPipeline.startVideo();

        while(opModeIsActive()) {
            double[] centerColor = hueTrackingPipeline.getCenterColorLab();
            double[] calibrationColor = hueTrackingPipeline.getSetpointLab();
            telemetry.addData("Calibration Color", (calibrationColor[0]) + " " + (calibrationColor[1]) + " " + (calibrationColor[2]));
            telemetry.addData("Center Color", (centerColor[0]) + " " + (centerColor[1]) + " " + (centerColor[2]));
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
