package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotClass;

public class Limelight {
    RobotClass robot;
    Limelight3A limelight;

    public Limelight(RobotClass robot){
        this.robot = robot;
        limelight = robot.limelight;
        initLimelight();
    }
    void initLimelight(){
        switchPipeline(0);
        limelight.start();
    }
    public void switchPipeline(int index){
        limelight.pipelineSwitch(index);
    }
    public LLResult getResult(){
       return limelight.getLatestResult();
    }
}
