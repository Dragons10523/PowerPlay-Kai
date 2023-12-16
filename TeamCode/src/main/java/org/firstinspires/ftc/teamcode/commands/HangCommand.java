package org.firstinspires.ftc.teamcode.commands;

import android.widget.Button;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class HangCommand extends CommandBase {
    ToolSubsystem toolSub;

    double hangPower;
    double hangStartPos;
    int power = 0;
    public HangCommand(ToolSubsystem tool, int power) {

        this.toolSub = tool;
        this.power = power;
    }
    @Override
    public void execute(){
        toolSub.hang(power);
    }
    @Override
    public boolean isFinished(){
        return power == 0;
    }
}