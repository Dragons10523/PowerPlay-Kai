package org.firstinspires.ftc.teamcode.vision;


import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Mushu;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

public class AprilTags extends SubsystemBase {
    AprilTagPipeline aprilTagPipeline;

    OpenCvCamera camera;
    Mushu mushu;

    AprilTagPose pose;


    ArrayList<AprilTagDetection> detections;

    public AprilTags(Mushu mushu, AprilTagPipeline aprilTagPipeline){
        this.mushu = mushu;
        this.aprilTagPipeline = aprilTagPipeline;
    }



    public ArrayList<AprilTagDetection> getDetections(){

        detections = aprilTagPipeline.getLatestDetections();
        return detections;

        // DEFAULT ID currently set to -1! returning -1 means that no detections were detected


    }
    public AprilTagPose getPose(){

        for(AprilTagDetection detection : detections){
            pose = detection.pose;

        }
        return pose;
    }

}