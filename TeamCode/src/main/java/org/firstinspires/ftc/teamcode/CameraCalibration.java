package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CameraCalibration")
public class CameraCalibration extends LinearOpMode {
    Ahi ahi;
    CalibrationPipeline calibrationPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        ahi = new Ahi(hardwareMap);

        calibrationPipeline = new CalibrationPipeline();

        ahi.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ahi.camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        ahi.camera.setPipeline(calibrationPipeline);

        waitForStart();

        while(!calibrationPipeline.isReady() && opModeIsActive()) {
            sleep(50);
        }

        while(opModeIsActive()) {
            double[] centerColor = calibrationPipeline.getColor();
            telemetry.addData("Real L*A*B*", (100*centerColor[0]/255) + " " + (centerColor[1]-128) + " " + (centerColor[2]-128));
            telemetry.addData("OpenCV L*A*B*", (centerColor[0]) + " " + (centerColor[1]) + " " + (centerColor[2]));
            telemetry.update();
            sleep(50);
        }
    }
}
