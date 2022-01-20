package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Optional;

public abstract class AbstractBarcode extends AbstractAutonomous {
    public HueTrackingPipeline hueTrackingPipeline;

    public void startOpenCV() {
        startOpenCV("tse");
    }

    public void startOpenCV(String calibFile) {
        telemetry.addLine("Parsing Calibration File");
        telemetry.update();

        double[] calibrationColor = parseCalibrationFile(calibFile);

        if(calibrationColor == null) { // Default to standard value if calibration failed
            hueTrackingPipeline = new HueTrackingPipeline();
            telemetry.addLine("Failed to retrieve calibration data from file");
        } else {
            hueTrackingPipeline = new HueTrackingPipeline(calibrationColor);
            telemetry.addData("Calibration Color", "[" + calibrationColor[0] + ", " + calibrationColor[1] + ", " + calibrationColor[2] + "]");
        }

        telemetry.update();
        
        telemetry.addLine("Opening Camera");
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

        telemetry.addLine("Setting Pipeline");
        telemetry.update();

        ahi.camera.setPipeline(hueTrackingPipeline);

        telemetry.addLine("Waiting for first frame");
        telemetry.update();

        while(!hueTrackingPipeline.isPipelineReady() && !isStopRequested()) {
            sleep(100);
        }

        telemetry.addLine("OpenCV Started");
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

        double[] calibrationData = parseCalibrationFile("fr8");
        Optional.ofNullable(calibrationData).orElse(new double[]{152, 135, 172}); // default value if null

        hueTrackingPipeline.setSetpointLab(calibrationData);

        ElapsedTime elapsedTime = new ElapsedTime();

        while(opModeIsActive()) {
            if(hueTrackingPipeline.getPixelCount() > 100) {
                double drift = hueTrackingPipeline.getAverageXPosition() - 0.5;
                drift *= 0.7;
                drive(0.8 + drift, 0.8 - drift);

                if (hueTrackingPipeline.getAverageYPosition() < 0.81) {
                    elapsedTime.reset();
                }

                if(elapsedTime.milliseconds() > 300) break;
            } else {
                drive(0, 0);
                sleep(33);

                if(elapsedTime.milliseconds() > 500) break;
            }
            telemetry.update();
        }

        drive(0, 0);

        hueTrackingPipeline.setSetpointLab(originalSetPoint);

        protectedSleep(500);
    }

    public void driveToShippingHub(FieldSide fieldSide) {
        double[] originalSetPoint = hueTrackingPipeline.getSetpointLab();

        double[] calibrationData = new double[3];

        switch(fieldSide) {
            case RED:
                calibrationData = parseCalibrationFile("red");
                Optional.ofNullable(calibrationData).orElse(new double[]{78, 165, 162}); // default value if null
                break;
            case BLUE:
                calibrationData = parseCalibrationFile("blu");
                Optional.ofNullable(calibrationData).orElse(new double[]{50, 134, 105});
                break;
        }

        hueTrackingPipeline.setSetpointLab(calibrationData);

        ElapsedTime elapsedTime = new ElapsedTime();

        while(opModeIsActive()) {
            if(hueTrackingPipeline.getPixelCount() > 100) {
                double drift = hueTrackingPipeline.getAverageXPosition() - 0.5;
                //drift *= 0.7;

                telemetry.addData("Drift", drift);

                double width = hueTrackingPipeline.getLargestRect().width;

                double speed = 250/width;

                telemetry.addData("Speed", speed);
                telemetry.update();

                speed = 0;

                drive(speed + drift, speed - drift);

                if(width >= 300) {
                    break;
                }

                elapsedTime.reset();
            } else {
                drive(0, 0);
                sleep(33);

                if(elapsedTime.milliseconds() > 500) break;
            }
        }

        drive(0, 0);

        hueTrackingPipeline.setSetpointLab(originalSetPoint);
    }

    public double[] parseCalibrationFile(String file) {
        double[] calibrationColor = new double[3];
        calibrationColor[2] = -1000;

        try {
            FileInputStream fis = new FileInputStream("/storage/emulated/0/FIRST/CalibrationData/" + file + ".dat"); // Get the color calibration
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

        if(calibrationColor[2] == -1000) {
            return null;
        } else {
            return calibrationColor;
        }
    }
}
