package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ColorEnum;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagCommand;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTags;
@Autonomous
public class AutoBlueBot extends CommandOpMode {
    Mushu mushu;
    AprilTags aprilTagsSub;
    AprilTagPipeline aprilTagPipeline;



    @Override
    public void initialize() {
        while(!isStarted()){
            sleep(20);

        }
        ColorEnum color = new ColorEnum(ColorEnum.Color.BLUE_DOWN);
        mushu = Mushu.GetInstance(this);
        aprilTagsSub = new AprilTags(mushu, aprilTagPipeline);

        schedule(new AutoDrive(mushu));

        schedule(new AprilTagCommand(mushu, telemetry, this::isStopRequested, aprilTagsSub));
        //schedule(new SequentialCommandGroup(new AutoDrive(mushu)));
    }

}
