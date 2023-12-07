package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.ButtonCall;
import org.firstinspires.ftc.teamcode.commands.DriveAuto;
import org.firstinspires.ftc.teamcode.vision.AprilTagCommand;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTags;

@Autonomous
public class Auto extends CommandOpMode {
    Mushu mushu;
    AprilTags aprilTagsSub;


    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);
       schedule(new AprilTagCommand(mushu, telemetry));



    }
    
}