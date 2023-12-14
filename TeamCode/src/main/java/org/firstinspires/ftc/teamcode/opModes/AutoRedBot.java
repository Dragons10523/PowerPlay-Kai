package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.ColorEnum;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.vision.AprilTagCommand;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTags;

public class AutoRedBot extends CommandOpMode {
    Mushu mushu;
    AprilTags aprilTagsSub;
    AprilTagPipeline aprilTagPipeline;


    @Override
    public void initialize() {
        ColorEnum color = new ColorEnum(ColorEnum.Color.RED_DOWN);
        mushu = Mushu.GetInstance(this);
        aprilTagsSub = new AprilTags(mushu, aprilTagPipeline);

        schedule(new AprilTagCommand(mushu, telemetry, this::isStopRequested, aprilTagsSub));
        //schedule(new SequentialCommandGroup(new AutoDrive(mushu)));
    }

}
