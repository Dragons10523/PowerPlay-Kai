package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Mushu;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class AprilTagCommand extends CommandBase {
    Mushu mushu;
    Telemetry telemetry;
    OpenCvCamera camera;
    BooleanSupplier isStopRequested;
    int numFramesWithoutDetection = 0;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTIONS_BEFORE_LOW_DECIMATION = 4;
    AprilTagPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    public AprilTagCommand(Mushu mushu, Telemetry telemetry, BooleanSupplier isStopRequested){
        this.mushu = mushu;
        this.telemetry = telemetry;
        this.isStopRequested = isStopRequested;


    }
    @Override
    public void initialize(){
        camera = mushu.camera;
        aprilTagDetectionPipeline = new AprilTagPipeline(0.166 ,0,0,0,0);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {

            }

        });


    }
    @Override
    public void execute(){
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

        if(detections != null)
        {
            telemetry.addData("FPS", camera.getFps());
            telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
            telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
            if(detections.size() == 0)
            {
                numFramesWithoutDetection++;
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTIONS_BEFORE_LOW_DECIMATION)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
        }
        else
        {
            numFramesWithoutDetection = 0;

            if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
            {
                aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
            }

            for(AprilTagDetection detection : detections)
            {
                Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
            }
        }
        telemetry.update();
    }
    @Override
    public void end(boolean interrupted){
        telemetry.clear();
        camera.closeCameraDevice();
    }
    @Override
    public boolean isFinished(){
        return isStopRequested.getAsBoolean();
    }

}
