package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public LLResult getResult(){
        return limelight.getLatestResult();
    }
}
