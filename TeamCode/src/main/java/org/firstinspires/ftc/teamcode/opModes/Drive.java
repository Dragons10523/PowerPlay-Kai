package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.commands.HangCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveWithSticks;
import org.firstinspires.ftc.teamcode.commands.Tool;
import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

@TeleOp
public class Drive extends CommandOpMode {
    Mushu mushu;
    ToolSubsystem m_toolSub;
    MecanumDriveSubsystems m_driveSub;

    InExtakeSub m_inExtake;



    //scheduler loops over initialize function (Why is it called initialize then IDK)

    @Override
    public void initialize() {
        //get instance makes sure there is only ever ONE object of mushu around
        mushu = Mushu.GetInstance(this);

        m_driveSub = new MecanumDriveSubsystems(mushu);
        m_toolSub = new ToolSubsystem(mushu.toolGamepad, mushu);
        m_inExtake = new InExtakeSub(mushu);


        schedule(new MecanumDriveWithSticks(mushu.mecanum, mushu.driverGamepad, m_driveSub, mushu, telemetry));

        schedule(new HangCommand(mushu.toolGamepad, m_toolSub, mushu));

        schedule(new Tool(mushu.toolGamepad,m_toolSub, mushu));

        schedule(new MecanumDriveWithSticks.ResetYaw(mushu, mushu.toolGamepad));

        schedule(new Tool.InExtake(mushu.toolGamepad, m_inExtake));

        schedule(new PerpetualCommand(new InstantCommand(() -> {

                })));
        //schedule(new Tool(mushu.toolGamepad, m_toolSub));

        //mushu.toolGamepad.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(new ConditionalCommand(
          //      schedule(new Tool(mushu.toolGamepad, m_toolSub)),
            //    schedule(new HangCommand(mushu.toolGamepad, m_toolSub)),

       // )




        //register gives priority to this main function to run these subsystems
        register(m_toolSub);
        register(m_driveSub);

    }

}
