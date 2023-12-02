package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.DriveAuto;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTags;
import org.openftc.easyopencv.OpenCvCamera;
@Autonomous
public class Auto extends CommandOpMode {
    Mushu mushu;
    AprilTags aprilTagsSub;


    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);

        schedule(new DriveAuto(mushu));
        //AprilTagPipeline aprilTagPipeline = new AprilTagPipeline(0,0,0,0,0);

        //aprilTagsSub = new AprilTags(mushu, aprilTagPipeline);
    }
    
}
