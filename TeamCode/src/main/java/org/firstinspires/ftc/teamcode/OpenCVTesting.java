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
            double[] centerColor = hueTrackingPipeline.getCenterColor();
            telemetry.addData("Center L*A*B*", centerColor[0] + " " + centerColor[1] + " " + centerColor[2]);
            telemetry.update();
            sleep(20);
        }

        ahi.camera.stopStreaming();
    }
}
