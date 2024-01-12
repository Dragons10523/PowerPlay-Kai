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
    Boolean isFinished = false;


    ColorPipeline.PieceLocation location;
    public AutoDrive(Mushu mushu, ColorPipeline.PieceLocation location){
        this.mushu = mushu;
        this.location = location;
    }
    public void execute(){
        if(location == ColorPipeline.PieceLocation.LEFT){
            mushu.mecanum.driveWithMotorPowers(.5,.5,.5,.5);
        }
        if(location == ColorPipeline.PieceLocation.CENTER){
            mushu.mecanum.driveWithMotorPowers(-.5,-.5, -.5, -.5);
        }
        if(location == ColorPipeline.PieceLocation.RIGHT){
            mushu.mecanum.driveWithMotorPowers(-.5, .5, -.5, .5);
        }
        if(location == null){
            mushu.intakeMotor.set(1);
        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        mushu.mecanum.stop();
        mushu.intakeMotor.set(0);
        this.cancel();

    }
    @Override
    public void end(boolean interrupted){
        mushu.mecanum.stop();
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }

}

