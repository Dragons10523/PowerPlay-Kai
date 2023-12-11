package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagCommand;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTags;

@Autonomous
public class Auto extends CommandOpMode {
    Mushu mushu;
    AprilTags aprilTagsSub;
    AprilTagPipeline aprilTagPipeline;


    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);
        aprilTagsSub = new AprilTags(mushu, aprilTagPipeline);

        schedule(new AprilTagCommand(mushu, telemetry, this::isStopRequested));
        //schedule(new SequentialCommandGroup(new AutoDrive(mushu)));
    }
    
}