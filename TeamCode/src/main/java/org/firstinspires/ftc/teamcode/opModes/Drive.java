package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveWithSticks;
import org.firstinspires.ftc.teamcode.commands.Tool;
import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

@TeleOp
public class Drive extends CommandOpMode {
    Mushu mushu;
    Tool m_tool;
    ToolSubsystem m_toolSub;
    MecanumDriveSubsystems m_driveSub;


    //scheduler loops over initialize function (Why is it called initialize then IDK)

    @Override
    public void initialize() {
        //get instance makes sure there is only ever ONE object of mushu around
        mushu = Mushu.GetInstance(this);

        m_driveSub = new MecanumDriveSubsystems(mushu.mecanum, mushu.driverGamepad);
        m_toolSub = new ToolSubsystem(mushu.toolGamepad);

        schedule(new MecanumDriveWithSticks(mushu.mecanum, mushu.driverGamepad, m_driveSub));
        schedule(new Tool(mushu.toolGamepad, m_toolSub));



        //register gives priority to this main function to run these subsystems
        register(m_toolSub);
        register(m_driveSub);
    }

}
