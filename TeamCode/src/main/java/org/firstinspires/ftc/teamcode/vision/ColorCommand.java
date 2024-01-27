package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.function.BooleanSupplier;

public class ColorCommand extends CommandBase {
    Mushu mushu;

    BooleanSupplier isStopRequested;
    ColorPipeline colorPipeline;
    boolean isOpened;
    Telemetry telemetry;
    OpenCvCamera camera;
    public ColorCommand(Mushu mushu, BooleanSupplier isStopRequested, Telemetry telemetry,  OpenCvCamera camera) {
        this.mushu = mushu;
        this.isStopRequested = isStopRequested;
        this.telemetry = telemetry;
        this.camera = camera;
    }
    public void initialize(){

        colorPipeline = new ColorPipeline(telemetry);
        camera.setPipeline(colorPipeline);
        telemetry.addData("isCameraOpened", isOpened);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
                isOpened = true;

                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("errorCode: " + errorCode);
                telemetry.update();
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

        telemetry.update();
        if(location == ColorPipeline.PieceLocation.LEFT){
            mushu.mecanum.driveWithMotorPowers(.5,.5,.5,.5);
            telemetry.update();
        }
        if(location == ColorPipeline.PieceLocation.CENTER){
            mushu.mecanum.driveWithMotorPowers(-.5,-.5, -.5, -.5);
            telemetry.update();
        }
        if(location == ColorPipeline.PieceLocation.RIGHT){
            mushu.mecanum.driveWithMotorPowers(-.5, .5, -.5, .5);
            telemetry.update();
        }
        if(location == null){
            mushu.intakeMotor.set(1);
            telemetry.update();
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
