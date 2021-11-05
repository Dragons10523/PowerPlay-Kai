package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvCamera;

public abstract class AbstractBarcode extends AbstractAutonomous {
    public HueTrackingPipeline hueTrackingPipeline;

    public void startOpenCV() {
        telemetry.addLine("Starting OpenCV");
        telemetry.update();

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

        while(!hueTrackingPipeline.isPipelineReady()) {
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
        double averageY = hueTrackingPipeline.getAverageYPosition();

        if(averageY < 0.333) {
            armPosition = ArmPosition.HIGH;
        } else if(averageY < 0.667) {
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
