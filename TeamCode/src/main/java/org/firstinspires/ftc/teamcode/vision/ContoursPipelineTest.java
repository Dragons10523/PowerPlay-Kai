package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorEnum;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ContoursPipelineTest extends CommandBase {
    Mushu mushu;
    OpenCvCamera camera;
    Telemetry telemetry;

    public ContoursPipelineTest(Mushu mushu, OpenCvCamera camera, Telemetry telemetry){
        this.mushu = mushu;
        this.camera = camera;
        this.telemetry = telemetry;
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
        telemetry.addData("PieceLocation", ColorPipeline.location);
        telemetry.addData("ColorEnum", ColorEnum.color);
        telemetry.update();

    }
    @Override
    public void end(boolean interrupted) {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
    @Override
    public boolean isFinished(){
        ColorPipeline colorPipeline = new ColorPipeline();

        try {
            return colorPipeline.confidence() < 40;
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
