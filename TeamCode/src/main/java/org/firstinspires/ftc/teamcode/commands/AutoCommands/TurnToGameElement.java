package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.vision.ColorPipelineBlue;
import org.firstinspires.ftc.teamcode.vision.ContoursPipelineTest;

import java.util.function.BooleanSupplier;

public class TurnToGameElement extends CommandBase
{
    Mushu mushu;
    //ColorPipeline colorPipeline;
    ColorPipelineBlue.PieceLocation location;
    CommandScheduler command;
    MecanumDriveSubsystems sub;
    Telemetry telemetry;
    BooleanSupplier isStopRequested;

    public TurnToGameElement(Mushu mushu, CommandScheduler commandScheduler, MecanumDriveSubsystems sub, Telemetry telemetry, BooleanSupplier isStopRequested){
        this.mushu = mushu;
        command = commandScheduler;
        this.sub = sub;
        this.telemetry = telemetry;
        this.isStopRequested = isStopRequested;
    }
    public void initialize(){

        location = ContoursPipelineTest.locationAtBeginning;

        if(location == ColorPipelineBlue.PieceLocation.LEFT){
            command.schedule(new SequentialCommandGroup(
                    new AutoTurn(90, sub, mushu, telemetry),
                    new AutoDrive(.3,-3, 0, sub, mushu, telemetry, isStopRequested)));


        }
        if(location == ColorPipelineBlue.PieceLocation.RIGHT){
            command.schedule(new SequentialCommandGroup(
                    new AutoTurn(270, sub, mushu, telemetry),
                    new AutoDrive(.3,3, 0, sub, mushu, telemetry, isStopRequested)));
        }
        else{
            command.schedule(new SequentialCommandGroup(
                    new AutoDrive(.3,-2, 0, sub, mushu, telemetry, isStopRequested),
                    new AutoTurn(0, sub, mushu, telemetry)));

        }

    }
    public void end(boolean interrupted) {
        mushu.mecanum.stop();

    }
}

