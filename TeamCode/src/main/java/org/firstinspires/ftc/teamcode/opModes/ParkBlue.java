package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoDrive;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoTurn;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTags;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class ParkBlue extends CommandOpMode {
    Mushu mushu;
    AprilTags aprilTagsSub;
    AprilTagPipeline aprilTagPipeline;
    OpenCvCamera camera;

    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);
        aprilTagsSub = new AprilTags(mushu, aprilTagPipeline);
        MecanumDriveSubsystems m_DriveSubsystem = new MecanumDriveSubsystems(mushu);
        InExtakeSub m_ExtakeSub = new InExtakeSub(mushu);

        schedule(new SequentialCommandGroup(
                 new AutoTurn(-90, m_DriveSubsystem, mushu),
                 new AutoDrive(.5,24,0, m_DriveSubsystem, mushu, telemetry, this::isStopRequested))
                .interruptOn(this::isStopRequested));
    }

}
