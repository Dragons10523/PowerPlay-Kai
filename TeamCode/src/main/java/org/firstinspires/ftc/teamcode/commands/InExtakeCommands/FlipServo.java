package org.firstinspires.ftc.teamcode.commands.InExtakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;

public class FlipServo extends CommandBase {
    InExtakeSub sub;
    public FlipServo(InExtakeSub sub){
        this.sub = sub;
    }
    public void execute(){
        sub.flipGate();
    }
    public boolean isFinished(){
        return true;
    }
}
