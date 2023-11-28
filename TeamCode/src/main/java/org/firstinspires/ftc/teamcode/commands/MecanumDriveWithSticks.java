package org.firstinspires.ftc.teamcode.commands;

import android.preference.PreferenceActivity;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    Telemetry telemetry;


    public MecanumDriveWithSticks(MecanumDrive mecanumDrive, GamepadEx driverGamepad, MecanumDriveSubsystems subsystem, Mushu mushu, Telemetry telemetry) {
        this.mecanum = mecanumDrive;
        this.driverGamepad = driverGamepad;
        this.m_driveSubsystem = subsystem;
        this.mushu = mushu;
        this.telemetry = telemetry;
    }

    @Override
    public void execute(){

        m_driveSubsystem.driveFieldCentric(forward, strafe, turn, heading);
        forward = -driverGamepad.getLeftY();
        strafe = -driverGamepad.getLeftX();
        turn = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        heading = mushu.getHeading();

        telemetry.addData("imu heading", mushu.getHeading());
        telemetry.update();

    }
}
