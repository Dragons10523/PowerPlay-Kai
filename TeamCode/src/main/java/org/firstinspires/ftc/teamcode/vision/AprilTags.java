package org.firstinspires.ftc.teamcode.vision;


import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.List;

public class AprilTags extends SubsystemBase {
    AprilTagPipeline aprilTagPipeline;

    OpenCvCamera camera;
    Mushu mushu;
    int id = -1;
    AprilTagPose pose;

    ArrayList<AprilTagDetection> detections;

    public AprilTags(Mushu mushu, AprilTagPipeline aprilTagPipeline){
        this.mushu = mushu;
        this.aprilTagPipeline = aprilTagPipeline;
    }



    public int getAprilTagID(){

        detections = aprilTagPipeline.getLatestDetections();

        for(AprilTagDetection detection : detections){
            id = detection.id;
        }
        // DEFAULT ID currently set to -1! returning -1 means that no detections were detected
        return id;
        // possible problem with only returning 1 id
    }
    public AprilTagPose getPose(){

        for(AprilTagDetection detection : detections){
            pose = detection.pose;

        }
        return pose;
    }

}