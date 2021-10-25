package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "OpenCVTest")
public class OpenCVTesting extends LinearOpMode {

    Ahi ahi;
    HueTrackingPipeline hueTrackingPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        ahi = new Ahi(hardwareMap);
        hueTrackingPipeline = new HueTrackingPipeline();

        ahi.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ahi.camera.startStreaming(160, 120);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        ahi.camera.setPipeline(hueTrackingPipeline);

        waitForStart();

        while(opModeIsActive()) {
            double[] centerHue = hueTrackingPipeline.getCenterColor();
            telemetry.addData("Center HSV", centerHue[0] + " " + centerHue[1] + " " + centerHue[2]);
            telemetry.update();
            sleep(20);
        }

        ahi.camera.stopStreaming();
    }
}
