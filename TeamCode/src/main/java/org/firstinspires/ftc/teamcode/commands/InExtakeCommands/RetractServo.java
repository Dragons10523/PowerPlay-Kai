package org.firstinspires.ftc.teamcode.commands.InExtakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;

public class RetractServo extends CommandBase {
    InExtakeSub sub;
    public RetractServo(InExtakeSub sub){
        this.sub = sub;
    }
    public void execute() {
        sub.retractGate();
    }
    public boolean isFinished(){
        return true;
    }
}
