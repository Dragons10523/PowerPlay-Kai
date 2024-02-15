package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.vision.ColorPipeline;
import org.firstinspires.ftc.teamcode.vision.ContoursPipelineTest;

import java.util.function.BooleanSupplier;

public class TurnToGameElement extends CommandBase
{
    Mushu mushu;
    //ColorPipeline colorPipeline;
    ColorPipeline.PieceLocation location;
    CommandScheduler command;
    MecanumDriveSubsystems sub;
    Telemetry telemetry;

    public TurnToGameElement(Mushu mushu, CommandScheduler commandScheduler, MecanumDriveSubsystems sub, Telemetry telemetry){
        this.mushu = mushu;
        command = commandScheduler;
        this.sub = sub;
        this.telemetry = telemetry;

    }
    public void initialize(){

        location = ContoursPipelineTest.locationAtBeginning;

        if(location == ColorPipeline.PieceLocation.LEFT){
            command.schedule(new AutoTurn(90, sub, mushu, telemetry));

        }
        if(location == ColorPipeline.PieceLocation.RIGHT){
            command.schedule(new AutoTurn(270, sub, mushu, telemetry));
        }

    }
    public void end(boolean interrupted) {
        mushu.mecanum.stop();

    }
}

