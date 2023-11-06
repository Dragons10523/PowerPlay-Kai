package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class Tool extends CommandBase {
    private final ToolSubsystem toolSubsystem;
    GamepadEx toolGamepad;

    public Tool(GamepadEx toolGamepad, ToolSubsystem subsystem){
        toolSubsystem = subsystem;
        this.toolGamepad = toolGamepad;
    }

    @Override
    public void initialize(){
        //hopefully this is the main function during teleOp
        //TODO: need to add tool at the end of the intake
        toolSubsystem.manualIntake();
        toolSubsystem.manualExtake();
    }


}
