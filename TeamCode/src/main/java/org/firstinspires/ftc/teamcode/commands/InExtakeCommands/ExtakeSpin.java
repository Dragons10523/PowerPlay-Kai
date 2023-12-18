package org.firstinspires.ftc.teamcode.commands.InExtakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;

public class ExtakeSpin extends CommandBase {
    InExtakeSub sub;
    double power;

    public ExtakeSpin(InExtakeSub sub, double power) {
        this.sub = sub;
        this.power = power;
    }

    public void execute(){
        sub.runEX(power);
    }
}

