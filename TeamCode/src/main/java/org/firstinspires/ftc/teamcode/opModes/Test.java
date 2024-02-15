package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mushu;
@TeleOp

public class Test extends CommandOpMode {
    @Override
    public void initialize() {
        Mushu mushu = Mushu.GetInstance(this);

        Button A = new GamepadButton(mushu.driverGamepad, GamepadKeys.Button.A);

        A.whenPressed(new org.firstinspires.ftc.teamcode.commands.Test(mushu, telemetry));

    }
}
