package org.firstinspires.ftc.teamcode.commands.InExtakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;

public class IntakeSpin extends CommandBase {
    GamepadEx gamepad;
    double intakePower;
    double extakePower;


    InExtakeSub sub;
    public IntakeSpin(GamepadEx gamepad, InExtakeSub sub){
        this.gamepad = gamepad;
        this.sub = sub;

    }
    @Override
    public void execute(){
        intakePower = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER); // INTAKE IS LEFT TRIGGER
        sub.runIN(intakePower);
    }

}
