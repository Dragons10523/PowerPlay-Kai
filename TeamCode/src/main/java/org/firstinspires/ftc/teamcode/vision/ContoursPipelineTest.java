package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorEnum;
import org.firstinspires.ftc.teamcode.Mushu;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ContoursPipelineTest extends CommandBase {
    Mushu mushu;
    OpenCvCamera camera;
    Telemetry telemetry;
    public static ColorPipeline.PieceLocation  locationAtBeginning;
    ColorEnum color;

    public ContoursPipelineTest(Mushu mushu, OpenCvCamera camera, Telemetry telemetry, ColorEnum color){
        this.mushu = mushu;
        this.camera = camera;
        this.telemetry = telemetry;
        this.color = color;
    }
    @Override
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
    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted) {
        locationAtBeginning = ColorPipeline.location;

        camera.stopStreaming();
        camera.closeCameraDevice();

    }
    @Override
    public boolean isFinished(){
        return true;
    }


}
