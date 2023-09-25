package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class ScoringMode extends CommandBase {
    private final ToolSubsystem m_toolSubsystem;
    private int xHex = 0;
    private int yHex = 0;
    private int zHex = 0;

    public ScoringMode(ToolSubsystem subsystem){
        m_toolSubsystem = subsystem;
        addRequirements(m_toolSubsystem);
    }



}
