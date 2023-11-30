package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;

public class AprilTagCommand extends CommandBase {

    Mushu mushu;
    AprilTags aprilTags;
    Telemetry telemetry;
    //TODO: set tagsize and calibration data for the camera
    public AprilTagCommand(Mushu mushu, AprilTags aprilTags, Telemetry telemetry){
        this.mushu = mushu;
        this.aprilTags = aprilTags;
        this.telemetry = telemetry;

    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        
    }
    public static class stuff{

    }

}
