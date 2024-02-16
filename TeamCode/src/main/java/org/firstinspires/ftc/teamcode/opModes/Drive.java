package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.DroneSub;
import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.commands.ButtonCall;
import org.firstinspires.ftc.teamcode.commands.HangCommand;
import org.firstinspires.ftc.teamcode.commands.InExtakeCommands.IntakeSpin;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveWithSticks;
import org.firstinspires.ftc.teamcode.commands.Tool;
import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

@TeleOp
public class Drive extends CommandOpMode {
    Mushu mushu;
    ToolSubsystem m_toolSub;
    MecanumDriveSubsystems m_driveSub;

    InExtakeSub m_inExtake;
    DroneSub m_droneSub;



    //scheduler loops over initialize function

    @Override
    public void initialize() {
        //get instance makes sure there is only ever ONE object of mushu around
        mushu = Mushu.GetInstance(this);

        /*mushu.frontRight.setInverted(true);
        mushu.backRight.setInverted(true);*/

        m_driveSub = new MecanumDriveSubsystems(mushu);
        m_toolSub = new ToolSubsystem(mushu.toolGamepad, mushu);
        m_inExtake = new InExtakeSub(mushu);
        m_droneSub  = new DroneSub(mushu);

        schedule(new MecanumDriveWithSticks(m_driveSub, mushu, telemetry));

        schedule(new ButtonCall(mushu, m_inExtake, m_toolSub, m_driveSub, m_droneSub));

        schedule(new Tool(mushu.toolGamepad,m_toolSub, mushu, telemetry));

        schedule(new IntakeSpin(mushu.toolGamepad, m_inExtake));

        //  schedule(new PerpetualCommand(new InstantCommand(() -> {

           //     })));

        //register gives priority to this main function to run these subsystems
        register(m_toolSub);
        register(m_driveSub);

    }

}
