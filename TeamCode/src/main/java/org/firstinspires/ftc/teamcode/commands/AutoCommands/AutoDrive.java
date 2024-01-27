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


    ColorPipeline.PieceLocation location = ColorPipeline.location;
    public AutoDrive(Mushu mushu){
        this.mushu = mushu;

    }
    public void execute(){



        mushu.mecanum.stop();
        mushu.intakeMotor.set(0);

        this.cancel();

    }
    @Override
    public void end(boolean interrupted){
        mushu.mecanum.stop();
        mushu.intakeMotor.set(0);

    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }

}

