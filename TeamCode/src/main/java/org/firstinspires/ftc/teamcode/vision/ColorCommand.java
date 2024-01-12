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
    ColorPipeline colorPipeline;
    public ColorCommand(Mushu mushu, BooleanSupplier isStopRequested) {
        this.mushu = mushu;
        this.isStopRequested = isStopRequested;
    }
    public void initialize(){
        camera = mushu.camera;

        colorPipeline = new ColorPipeline();
        camera.setPipeline(colorPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("errorCode: " + errorCode);
            }

        });

    }

    public void execute(){
        ColorPipeline.PieceLocation location = colorPipeline.getLocation();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        if(location == ColorPipeline.PieceLocation.LEFT){
            mushu.mecanum.driveWithMotorPowers(.5,.5,.5,.5);
        }
        if(location == ColorPipeline.PieceLocation.CENTER){
            mushu.mecanum.driveWithMotorPowers(-.5,-.5, -.5, -.5);
        }
        if(location == ColorPipeline.PieceLocation.RIGHT){
            mushu.mecanum.driveWithMotorPowers(-.5, .5, -.5, .5);
        }
        if(location == null){
            mushu.intakeMotor.set(1);
        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        mushu.mecanum.stop();
        mushu.intakeMotor.set(0);
        this.cancel();

    }

    public void end(boolean interrupted){
        camera.closeCameraDevice();

    }
}
