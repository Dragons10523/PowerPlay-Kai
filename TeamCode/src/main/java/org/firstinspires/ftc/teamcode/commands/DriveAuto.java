package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mushu;

public class DriveAuto extends CommandBase {
    Mushu mushu;
    public enum TeamColor{
        BLUE,
        RED
    }
    TeamColor color;


    public DriveAuto(Mushu mushu){
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

    }
    public boolean isFinished(){
        return true;
    }

}
