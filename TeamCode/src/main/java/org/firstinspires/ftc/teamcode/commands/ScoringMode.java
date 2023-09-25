package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class ScoringMode extends CommandBase {
    private final ToolSubsystem m_toolSubsystem;
    private int xHex = 0; //   https://i.stack.imgur.com/n9nmA.jpg
    private int yHex = 0; //  x direction = u
    private int zHex = 0; //  y direction = v
                          //  z direction = w
                          //
    public ScoringMode(ToolSubsystem subsystem){
        m_toolSubsystem = subsystem;
        addRequirements(m_toolSubsystem);
    }

    public void gridinator(GamepadEx driverGamepad){
        boolean up = driverGamepad.gamepad.dpad_up;
        boolean right = driverGamepad.gamepad.dpad_right;
        boolean left = driverGamepad.gamepad.dpad_left;
        if(up){
            yHex++;
        }
        if(right){
            xHex++;
        }
        if(left){
            zHex++;
        }
    }

}
