package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

public class Tool extends CommandBase {
    private final ToolSubsystem m_toolSubsystem;

    public Tool(ToolSubsystem subsystem){
        m_toolSubsystem = subsystem;
        addRequirements(m_toolSubsystem);
    }
    @Override
    public void initialize(){
        m_toolSubsystem.
    }
}
