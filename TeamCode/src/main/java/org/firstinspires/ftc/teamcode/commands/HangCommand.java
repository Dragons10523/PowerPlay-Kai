package org.firstinspires.ftc.teamcode.commands;

import android.widget.Button;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class HangCommand extends CommandBase {
    GamepadEx toolGamepad;
    ToolSubsystem tool;
    Mushu mushu;
    double hangPower;
    double hangStartPos;
    public HangCommand(GamepadEx toolGamepad, ToolSubsystem tool, Mushu mushu) {
        this.toolGamepad = toolGamepad;
        this.mushu = mushu;
        this.tool = tool;
    }
    public void initialize(){
        hangStartPos = mushu.hangMotor.getCurrentPosition();
    }
    @Override
    public void execute(){
        hangPower = toolGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - toolGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        if(hangStartPos >= mushu.hangMotor.getCurrentPosition() + 5){
           tool.hang(Math.max(hangPower, 0));
        }
        else{
            tool.hang(hangPower);
        }

    }
}