package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class DriveWithSticks extends CommandBase {
    DifferentialDrive driveTrain;
    GamepadEx driverGamepad;

    public DriveWithSticks(DifferentialDrive driveTrain, GamepadEx driverGamepad)
    {
        this.driveTrain = driveTrain;
        this.driverGamepad = driverGamepad;
    }

    @Override
    public void execute() {
        driveTrain.tankDrive(driverGamepad.getLeftY(), driverGamepad.getRightY());
    }
}
