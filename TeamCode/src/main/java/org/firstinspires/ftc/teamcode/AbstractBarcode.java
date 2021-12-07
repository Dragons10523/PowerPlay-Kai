package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

public abstract class AbstractBarcode extends AbstractAutonomous {
    public HueTrackingPipeline hueTrackingPipeline;

    public void startOpenCV() {
        telemetry.addLine("Starting OpenCV");
        //telemetry.update();

        double[] calibrationColor = new double[3];
        calibrationColor[2] = -1000;

        FileInputStream fis = null;
        try {
            fis = new FileInputStream("/storage/emulated/0/FIRST/color.dat"); // Get the color calibration
            for (int i = 0; i < 3; i++) {
                byte[] byteArray = new byte[8];
                int bytesRead = fis.read(byteArray, 0, 8);

                if(bytesRead == -1) {
                    break;
                }

                ByteBuffer byteBuffer = ByteBuffer.wrap(byteArray);
                calibrationColor[i] = byteBuffer.getDouble();
            }
            fis.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        if(calibrationColor[2] == -1000) { // Default to standard value if calibration failed
            hueTrackingPipeline = new HueTrackingPipeline();
            telemetry.addLine("Failed to retrieve calibration data from file");
        } else {
            hueTrackingPipeline = new HueTrackingPipeline(calibrationColor);
        }

        telemetry.addData("Calibration Color", "[" + calibrationColor[0] + ", " + calibrationColor[1] + ", " + calibrationColor[2] + "]");
        telemetry.update();

        ahi.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ahi.camera.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        ahi.camera.setPipeline(hueTrackingPipeline);

        while(!hueTrackingPipeline.isPipelineReady() && opModeIsActive()) {
            sleep(100);
        }

        telemetry.clearAll();

        telemetry.update();
    }

    public void stopOpenCV() {
        ahi.camera.stopStreaming();
        ahi.camera.closeCameraDevice();
    }

    public ArmPosition getFieldOrientation() {
        ArmPosition armPosition;
        double averageX = hueTrackingPipeline.getAverageXPosition();

        if(averageX < 0.333) {
            armPosition = ArmPosition.HIGH;
        } else if(averageX < 0.667) {
            armPosition = ArmPosition.MED;
        } else {
            armPosition = ArmPosition.LOW;
        }

        return armPosition;
    }

    /*@Override
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
            double[] centerColor = hueTrackingPipeline.getCenterColorLab();
            double[] centerColorVanilla = hueTrackingPipeline.getCenterColorVanilla();
            telemetry.addData("Center Vanilla", centerColorVanilla[0] + " " + centerColorVanilla[1] + " " + centerColorVanilla[2]);
            telemetry.addData("Center L*A*B*", (100*centerColor[0]/255) + " " + (centerColor[1]-128) + " " + (centerColor[2]-128));
            telemetry.addData("Tracking Pipeline Overhead", ahi.camera.getTotalFrameTimeMs());
            telemetry.update();
            sleep(20);
        }

        ahi.camera.stopStreaming();
        ahi.camera.closeCameraDevice();
    }*/
}
