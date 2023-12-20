package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ColorEnum;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.function.BooleanSupplier;

public class ColorCommand extends CommandBase {
    Mushu mushu;
    OpenCvCamera camera;
    BooleanSupplier isStopRequested;
    public ColorCommand(Mushu mushu, BooleanSupplier isStopRequested) {
        this.mushu = mushu;
        this.isStopRequested = isStopRequested;
    }

    public void initialize(){

        camera = mushu.camera;
        ColorPipeline colorPipeline = new ColorPipeline();
        camera.setPipeline(colorPipeline);

        //open Camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {

                System.out.println("errorCode:" + errorCode);
            }

        });
        //waits for detection or for 7.5 seconds
        waitForColorDetection(colorPipeline);

        ColorPipeline.PieceLocation location = colorPipeline.getLocation();

        if(location == ColorPipeline.PieceLocation.LEFT){
            new AutoDrive(mushu, location);
        }
        if(location == ColorPipeline.PieceLocation.RIGHT){
            new AutoDrive(mushu, location);
        }
        if(location == ColorPipeline.PieceLocation.CENTER){
            new AutoDrive(mushu, location);
        }
        else{
            mushu.mecanum.driveWithMotorPowers(.5,.5,.5,.5);
        }

        end(true);


    }

    public boolean isFinished(){

        return isStopRequested.getAsBoolean();
    }
    public void end(boolean Interrupted){
        mushu.mecanum.stop();
        camera.closeCameraDevice();
        this.cancel();

    }
    public void waitForColorDetection(ColorPipeline colorPipeline){
        int loopcounter = 0;
        while(loopcounter > 150 || colorPipeline.getLocation() == null){
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            loopcounter++;
        }
    }

}
