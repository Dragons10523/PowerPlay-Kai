package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Rect;
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
                ahi.camera.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
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

    public ArmPosition getFieldOrientation(FieldSide fieldSide) {
        ArmPosition armPosition;
        double averageX = hueTrackingPipeline.getAverageXPosition();

        double adjust = fieldSide == FieldSide.RED ? 0.2 : -0.333;

        telemetry.addData("X", averageX);

        if(averageX > 0.667 + adjust) {
            armPosition = ArmPosition.HIGH_FORE;
        } else if(averageX > 0.333 + adjust) {
            armPosition = ArmPosition.MED_FORE;
        } else {
            armPosition = ArmPosition.LOW_FORE;
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
            if(hueTrackingPipeline.getPixelCount() > 5) {
                telemetry.addData("Pixels", hueTrackingPipeline.getPixelCount());

                double drift = hueTrackingPipeline.getAverageXPosition() - 0.5;
                drift *= 0.9;

                double speed = Math.min(1.2, 1.55-hueTrackingPipeline.getAverageYPosition());

                drive(speed + drift, speed - drift);

                telemetry.addData("Y", hueTrackingPipeline.getAverageYPosition());

                if (hueTrackingPipeline.getAverageYPosition() < 0.85) {
                    elapsedTime.reset();
                }

                if(elapsedTime.milliseconds() > 150) break;
            } else {
                drive(0, 0);
                sleep(33);
                if(elapsedTime.milliseconds() > 100) break;
            }
            telemetry.update();
        }

        drive(0, 0);

        hueTrackingPipeline.setSetpointLab(originalSetPoint);

        protectedSleep(500);
    }

    public void driveToShippingHub(FieldSide fieldSide) {
        ElapsedTime timer = new ElapsedTime();

        hueTrackingPipeline.setRectProc(true);
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
            if(timer.milliseconds() > 2000) {
                drive(0, 0);
                break;
            }

            if(hueTrackingPipeline.getPixelCount() > 5) {
                Rect rect = hueTrackingPipeline.getLargestRect();
                double averageX = hueTrackingPipeline.getAverageXPosition();//(rect.x + rect.width)/320f;

                double drift = averageX - 0.5;
                drift *= 2.5;

                double height = rect.height;

                double speed = 13/height;

                speed = Math.min(speed, 1);

                drive(speed + drift, speed - drift);

                if(height >= 56 ) {
                    telemetry.addData("Height Break", height);
                    drive(0, 0);
                    break;
                }

                elapsedTime.reset();
            } else {
                drive(0, 0);
                sleep(33);

                if(elapsedTime.milliseconds() > 200) {
                    telemetry.addData("Pixel Break", hueTrackingPipeline.getPixelCount());
                    break;
                };
            }
        }
        telemetry.update();

        hueTrackingPipeline.setRectProc(false);

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
