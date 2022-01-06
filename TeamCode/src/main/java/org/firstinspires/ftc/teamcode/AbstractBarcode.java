package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

public abstract class AbstractBarcode extends AbstractAutonomous {
    public HueTrackingPipeline hueTrackingPipeline;

    public void startOpenCV() {
        telemetry.addLine("Starting OpenCV");

        double[] calibrationColor = new double[3];
        calibrationColor[2] = -1000;

        try {
            FileInputStream fis = new FileInputStream("/storage/emulated/0/FIRST/color.dat"); // Get the color calibration
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
                ahi.camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        ahi.camera.setPipeline(hueTrackingPipeline);

        while(!hueTrackingPipeline.isPipelineReady() && !isStopRequested()) {
            sleep(100);
        }

        telemetry.clearAll();

        telemetry.update();
    }

    public void stopOpenCV() {
        hueTrackingPipeline.stopVideo();
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

    public void driveToFreight() {
        double[] originalSetPoint = hueTrackingPipeline.getSetpointLab();
        hueTrackingPipeline.setSetpointLab(new double[]{152, 135, 172});

        ElapsedTime elapsedTime = new ElapsedTime();

        while(opModeIsActive()) {
            telemetry.addLine("looping");
            if(hueTrackingPipeline.getPixelCount() > 100) {
                double drift = hueTrackingPipeline.getAverageXPosition() - 0.5;
                drift *= 0.7;

                telemetry.addData("Drift", drift);
                drive(0.8 + drift, 0.8 - drift);

                if (hueTrackingPipeline.getAverageYPosition() < 0.81) {
                    elapsedTime.reset();
                }

                if(elapsedTime.milliseconds() > 300) break;
            } else {
                drive(0, 0);
                telemetry.addLine("not enough pixels");
                sleep(33);

                if(elapsedTime.milliseconds() > 500) break;
            }
            telemetry.update();
        }
        telemetry.addLine("stopped");
        telemetry.update();

        drive(0, 0);

        hueTrackingPipeline.setSetpointLab(originalSetPoint);

        protectedSleep(500);
    }

    public void driveToShippingHub(FieldSide fieldSide) {
        double[] originalSetPoint = hueTrackingPipeline.getSetpointLab();

        switch(fieldSide) {
            case RED:
                hueTrackingPipeline.setSetpointLab(new double[]{78, 165, 162});
            case BLUE:
                hueTrackingPipeline.setSetpointLab(new double[]{50, 134, 105});
        }

        while(opModeIsActive()) {
            if(hueTrackingPipeline.getPixelCount() > 100) {
                double drift = hueTrackingPipeline.getAverageXPosition() - 0.5;

                drive(0.7 + drift, 0.7 - drift);

                if (hueTrackingPipeline.getPixelCount() > 5500) {
                    break;
                }
            } else {
                sleep(33);
                drive(0, 0);
            }
        }

        drive(0, 0);

        hueTrackingPipeline.setSetpointLab(originalSetPoint);
    }
}
