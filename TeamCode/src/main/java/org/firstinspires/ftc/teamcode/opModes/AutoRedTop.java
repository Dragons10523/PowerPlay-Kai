package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColorEnum;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoDrive;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoTurn;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.TurnToGameElement;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.UnloadPixel;

import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTags;
import org.firstinspires.ftc.teamcode.vision.ContoursPipelineTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous

public class AutoRedTop extends CommandOpMode {
    Mushu mushu;
    AprilTags aprilTagsSub;
    AprilTagPipeline aprilTagPipeline;
    OpenCvCamera camera;


    @Override
    public void initialize() {
        CommandScheduler command = CommandScheduler.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        new ColorEnum(ColorEnum.Color.RED_UP);
        mushu = Mushu.GetInstance(this);
        aprilTagsSub = new AprilTags(mushu, aprilTagPipeline);

        MecanumDriveSubsystems m_DriveSubsystem = new MecanumDriveSubsystems(mushu);
        InExtakeSub m_ExtakeSub = new InExtakeSub(mushu);

        new AutoDrive(m_DriveSubsystem, mushu);
        new AutoTurn(m_DriveSubsystem, mushu);


        //schedule(new AprilTagCommand(mushu, telemetry, this::isStopRequested, aprilTagsSub));

        schedule(new SequentialCommandGroup(new ContoursPipelineTest(mushu, camera, telemetry)), new AutoDrive(1, 24, 0),
                new TurnToGameElement(mushu, command), new UnloadPixel(mushu, m_ExtakeSub));
    }

}
