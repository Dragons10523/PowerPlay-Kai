package org.firstinspires.ftc.teamcode.FeedForward;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.ArrayList;

public class PathBuilder {
    SparkFunOTOS.Pose2D startPos = new SparkFunOTOS.Pose2D(0,0,0);
    SparkFunOTOS.Pose2D endPos = new SparkFunOTOS.Pose2D(10,0,0);

    ArrayList<SparkFunOTOS.Pose2D> pathPoints = new ArrayList<>();

    public PathBuilder(){
        
    }

}
