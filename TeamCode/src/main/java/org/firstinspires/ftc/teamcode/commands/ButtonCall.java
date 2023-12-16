package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class ButtonCall extends CommandBase {
    Mushu mushu;
    Button Dpad_DOWN_Tool, Dpad_DOWN_Drive;
    Button Dpad_UP_Tool, Dpad_UP_Drive;
    Button Left_Bumper;
    Button Right_Bumper;
    Button BACK;
    Button A;

    InExtakeSub m_InExtakeSub;
    ToolSubsystem m_toolSub;
    public ButtonCall(Mushu mushu, InExtakeSub sub, ToolSubsystem toolSub){
        this.mushu = mushu;
        m_InExtakeSub = sub;
        m_toolSub = toolSub;

    }
    public void initialize(){
        Dpad_DOWN_Tool = new GamepadButton(
                mushu.toolGamepad, GamepadKeys.Button.DPAD_DOWN
        );
        Dpad_UP_Tool = new GamepadButton(
                mushu.toolGamepad, GamepadKeys.Button.DPAD_UP
        );
        Left_Bumper = new GamepadButton(
                mushu.toolGamepad, GamepadKeys.Button.LEFT_BUMPER
        );
        Right_Bumper = new GamepadButton(
                mushu.toolGamepad, GamepadKeys.Button.RIGHT_BUMPER
        );
        BACK = new GamepadButton(
                mushu.driverGamepad, GamepadKeys.Button.BACK
        );
        Dpad_UP_Drive = new GamepadButton(
                mushu.driverGamepad, GamepadKeys.Button.DPAD_UP
        );
        Dpad_DOWN_Drive = new GamepadButton(
                mushu.driverGamepad, GamepadKeys.Button.DPAD_DOWN
        );
        A = new GamepadButton(
                mushu.toolGamepad, GamepadKeys.Button.A
        );

    }
    public void execute(){
      //Dpad_DOWN.whenPressed(new InstantCommand())
      Dpad_DOWN_Tool.whenPressed(new Tool.flipServo(m_InExtakeSub));
      Dpad_UP_Tool.whenPressed(new Tool.retractServo(m_InExtakeSub));
      Right_Bumper.whenHeld(new Tool.extakeSpin(m_InExtakeSub, .75));
      Right_Bumper.whileHeld(new Tool.extakeSpin(m_InExtakeSub, .75)).whenReleased(new Tool.extakeSpin(m_InExtakeSub, 0));
      Left_Bumper.whileHeld(new Tool.extakeSpin(m_InExtakeSub, -.75)).whenReleased(new Tool.extakeSpin(m_InExtakeSub, 0));
      BACK.whenPressed(new MecanumDriveWithSticks.ResetYaw(mushu));
      Dpad_DOWN_Drive.whileHeld(new HangCommand(m_toolSub, -1)).whenReleased(new HangCommand(m_toolSub, 0));
      Dpad_UP_Drive.whileHeld(new HangCommand(m_toolSub, 1)).whenReleased(new HangCommand(m_toolSub, 0));
    }
}
