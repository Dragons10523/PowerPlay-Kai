package org.firstinspires.ftc.teamcode.commands;

import android.preference.PreferenceActivity;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

public class MecanumDriveWithSticks extends CommandBase {
    Mushu mushu;
    MecanumDrive mecanum;
    GamepadEx driverGamepad;
    MecanumDriveSubsystems m_driveSubsystem;

    double forward;
    double strafe;
    double turn;
    double heading;


    public MecanumDriveWithSticks(MecanumDrive mecanumDrive, GamepadEx driverGamepad, MecanumDriveSubsystems subsystem) {
        this.mecanum = mecanumDrive;
        this.driverGamepad = driverGamepad;
        this.m_driveSubsystem = subsystem;
    }

    @Override
    public void execute(){

        m_driveSubsystem.driveFieldCentric(forward, strafe, turn, heading);
        forward = driverGamepad.getLeftY();
        strafe = driverGamepad.getLeftX();
        turn = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        heading = mushu.getHeading();
    }
}
