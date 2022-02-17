package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

public abstract class AbstractCalibrate extends LinearOpMode {
    Ahi ahi;
    CalibrationPipeline calibrationPipeline;

    public void calibrate(String file) throws InterruptedException {
        ahi = new Ahi(hardwareMap);

        calibrationPipeline = new CalibrationPipeline();

        ahi.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ahi.camera.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error code " + errorCode + " while opening the camera");
                telemetry.update();
            }
        });

        ahi.camera.setPipeline(calibrationPipeline);

        waitForStart();

        while(!calibrationPipeline.isReady() && opModeIsActive()) {
            sleep(50);
        }

        double[] centerColor = calibrationPipeline.getColor();

        int frame = 0;
        while(opModeIsActive()) {
            double[] color = calibrationPipeline.getColor();
            centerColor[1] += color[1];
            centerColor[2] += color[2];

            centerColor[1] /= 2;
            centerColor[2] /= 2;

            telemetry.addData("Real L*A*B*", (100*centerColor[0]/255) + " " + (centerColor[1]-128) + " " + (centerColor[2]-128));
            telemetry.addData("OpenCV L*A*B*", (centerColor[0]) + " " + (centerColor[1]) + " " + (centerColor[2]));
            telemetry.update();

            while(ahi.camera.getFrameCount() <= frame) {
                frame = ahi.camera.getFrameCount();
                sleep(1);
            }
        }

        centerColor[0] = 0;

        new File("/storage/emulated/0/FIRST/CalibrationData/").mkdirs();

        try {
            FileOutputStream fos = new FileOutputStream("/storage/emulated/0/FIRST/CalibrationData/" + file + ".dat");
            ByteBuffer data = (ByteBuffer) ByteBuffer.allocate(8 * 3).putDouble(centerColor[0]).putDouble(centerColor[1]).putDouble(centerColor[2]).rewind();
            byte[] dataArray = new byte[8*3];
            data.get(dataArray, 0, dataArray.length);
            fos.write(dataArray);
            fos.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
