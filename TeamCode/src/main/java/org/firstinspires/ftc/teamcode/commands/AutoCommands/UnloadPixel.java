package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;



public class UnloadPixel extends CommandBase {

    Mushu mushu;
    InExtakeSub sub;

    public UnloadPixel(Mushu mushu, InExtakeSub sub){
        this.mushu = mushu;
        this.sub = sub;
    }

    public void execute(){
        sub.runIN(1);
        this.cancel();
    }
    public void end(boolean interrupted){
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sub.runIN(0);
    }


}
