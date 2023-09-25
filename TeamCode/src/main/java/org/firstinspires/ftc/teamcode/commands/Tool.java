package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class Tool extends CommandBase {
    private final ToolSubsystem m_toolSubsystem;

    public Tool(ToolSubsystem subsystem){
        m_toolSubsystem = subsystem;
        addRequirements(m_toolSubsystem);
    }
    @Override
    public void initialize(){

    }
    @Override
    public boolean isFinished(){

        return(true);

    }

}
