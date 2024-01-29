package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.vision.ColorPipeline;

public class TurnToGameElement extends CommandBase
{
    Mushu mushu;
    //ColorPipeline colorPipeline;
    ColorPipeline.PieceLocation location;
    CommandScheduler command;
    MecanumDriveSubsystems sub;

    public TurnToGameElement(Mushu mushu, CommandScheduler commandScheduler, MecanumDriveSubsystems sub){
        this.mushu = mushu;
        command = commandScheduler;
        this.sub = sub;

    }
    public void initialize(){
        location = ColorPipeline.location;
    }
    public void execute(){
        if(location == ColorPipeline.PieceLocation.LEFT){
            command.schedule(new AutoTurn(-90, sub, mushu));
        }
        if(location == ColorPipeline.PieceLocation.RIGHT){
            command.schedule(new AutoTurn(-90, sub, mushu));
        }
        this.cancel();
    }




}

