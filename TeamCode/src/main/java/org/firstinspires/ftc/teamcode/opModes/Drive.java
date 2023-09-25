package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveWithSticks;
import org.firstinspires.ftc.teamcode.commands.Tool;
import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class Drive extends CommandOpMode {
    Mushu mushu;
    Tool m_tool;
    ToolSubsystem m_toolSub;
    MecanumDriveSubsystems driveSub;

    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);
        schedule(new MecanumDriveWithSticks(mushu.mecanum, mushu.driverGamepad, driveSub));

        register(m_toolSub);

    }
}
