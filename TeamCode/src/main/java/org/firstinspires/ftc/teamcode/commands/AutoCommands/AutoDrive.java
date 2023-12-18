package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mushu;

public class AutoDrive extends CommandBase {
    Mushu mushu;
    public enum TeamColor{
        BLUE,
        RED
    }
    TeamColor color;



    public AutoDrive(Mushu mushu){
        this.mushu = mushu;

    }
    public void initialize(){

        mushu.mecanum.driveWithMotorPowers(.5,.5,.5,.5);
        try {
            Thread.sleep(2000);
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

