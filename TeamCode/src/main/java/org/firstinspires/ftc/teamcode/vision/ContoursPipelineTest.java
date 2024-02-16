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
    public static ColorPipelineBlue.PieceLocation  locationAtBeginning;
    ColorEnum color;

    public ContoursPipelineTest(Mushu mushu, OpenCvCamera camera, Telemetry telemetry, ColorEnum color){
        this.mushu = mushu;
        this.camera = camera;
        this.telemetry = telemetry;
        this.color = color;
    }
    @Override
    public void initialize(){
        if(ColorEnum.color == ColorEnum.Color.RED_UP || ColorEnum.color == ColorEnum.Color.RED_DOWN){
            camera.setPipeline(new ColorPipelineRed(telemetry));
        }
        else{
            camera.setPipeline(new ColorPipelineBlue(telemetry));
        }

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
        locationAtBeginning = ColorPipelineBlue.location;

        camera.stopStreaming();
        camera.closeCameraDevice();

    }
    @Override
    public boolean isFinished(){
        return true;
    }


}
