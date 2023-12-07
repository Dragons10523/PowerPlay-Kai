package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;

public class ButtonCall extends CommandBase {
    Mushu mushu;
    Button Dpad_DOWN;
    Button Dpad_UP;
    Button Left_Bumper;
    Button Right_Bumper;
    Button BACK;
    Button A;
    Telemetry telemetry;
    InExtakeSub m_InExtakeSub;
    public ButtonCall(Mushu mushu, InExtakeSub sub, Telemetry telemetry){
        this.mushu = mushu;
        m_InExtakeSub = sub;
        this.telemetry = telemetry;
    }
    public void initialize(){
        Dpad_DOWN = new GamepadButton(
                mushu.toolGamepad, GamepadKeys.Button.DPAD_DOWN
        );
        Dpad_UP = new GamepadButton(
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
        A = new GamepadButton(
                mushu.toolGamepad, GamepadKeys.Button.A
        );

    }
    public void execute(){
      //Dpad_DOWN.whenPressed(new InstantCommand())
      Dpad_DOWN.whenPressed(new Tool.flipServo(m_InExtakeSub));
      Dpad_UP.whenPressed(new Tool.retractServo(m_InExtakeSub));
      Right_Bumper.whileHeld(new Tool.extakeSpin(m_InExtakeSub, .75)).whenReleased(new Tool.extakeSpin(m_InExtakeSub, 0));
      Left_Bumper.whileHeld(new Tool.extakeSpin(m_InExtakeSub, -.75)).whenReleased(new Tool.extakeSpin(m_InExtakeSub, 0));
      BACK.whenPressed(new MecanumDriveWithSticks.ResetYaw(mushu, telemetry));
    }
}
