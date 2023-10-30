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
    MecanumDriveSubsystems m_driveSub;



    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);
        schedule(new MecanumDriveWithSticks(mushu.mecanum, mushu.driverGamepad, m_driveSub));

        m_driveSub = new MecanumDriveSubsystems(hardwareMap, "FrontLeft","FrontRight","BackLeft","BackRight", mushu.mecanum);
        m_toolSub = new ToolSubsystem(hardwareMap, "intakeServo", "arm", "omniServo");
        register(m_toolSub);
        register(m_driveSub);
    }

}
