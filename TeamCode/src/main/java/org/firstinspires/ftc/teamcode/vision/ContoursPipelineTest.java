package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.function.BooleanSupplier;

public class ContoursPipelineTest extends CommandBase {
    Mushu mushu;
    OpenCvCamera camera;
    BooleanSupplier isOpModeActive;
    Telemetry telemetry;
    public ContoursPipelineTest(Mushu mushu, OpenCvCamera camera, BooleanSupplier isOpModeActive, Telemetry telemetry){
        this.mushu = mushu;
        this.camera = camera;
        this.isOpModeActive = isOpModeActive;
        this.telemetry = telemetry;
    }
    public void initialize(){
        camera.setPipeline(new ColorPipeline(telemetry));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public void execute(){
        while(isOpModeActive.getAsBoolean()){

        }
        this.cancel();
    }
    public void end(Boolean Interrupted){
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
