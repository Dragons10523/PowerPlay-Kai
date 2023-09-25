package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

public class MecanumDriveWithSticks extends CommandBase {
    MecanumDrive mecanum;
    GamepadEx driverGamepad;

    public MecanumDriveWithSticks(MecanumDrive mecanumDrive, GamepadEx driverGamepad, MecanumDriveSubsystems subsystem) {

        this.mecanum = mecanumDrive;
        this.driverGamepad = driverGamepad;
    }

   

}
