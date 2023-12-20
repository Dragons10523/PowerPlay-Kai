package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.vision.ColorPipeline;

public class AutoDrive extends CommandBase {
    Mushu mushu;
    public enum TeamColor{
        BLUE,
        RED
    }
    TeamColor color;


    ColorPipeline.PieceLocation location;
    public AutoDrive(Mushu mushu, ColorPipeline.PieceLocation location){
        this.mushu = mushu;
        this.location = location;
    }
    public void initialize(){
        if(location == ColorPipeline.PieceLocation.LEFT){
            mushu.mecanum.driveWithMotorPowers(.5,.5,.5,.5);
        }
        if(location == ColorPipeline.PieceLocation.CENTER){
            mushu.mecanum.driveWithMotorPowers(-.5,-.5, -.5, -.5);
        }
        if(location == ColorPipeline.PieceLocation.RIGHT){
            mushu .mecanum.driveWithMotorPowers(-.5, .5, -.5, .5);
        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        mushu.mecanum.stop();
        end(true);

    }
    @Override
    public void end(boolean interrupted){
        mushu.mecanum.driveWithMotorPowers(0,0,0,0);
        mushu.mecanum.stop();
    }

}

