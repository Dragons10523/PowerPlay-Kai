package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

public class MecanumDriveWithSticks extends CommandBase {
    Mushu mushu;
    MecanumDriveSubsystems m_driveSubsystem;

    double backward;
    double strafe;
    double turn;
    double heading;
    Telemetry telemetry;

    public MecanumDriveWithSticks(MecanumDriveSubsystems subsystem, Mushu mushu, Telemetry telemetry) {
        this.m_driveSubsystem = subsystem;
        this.mushu = mushu;
        this.telemetry = telemetry;
    }
    public void initialize(){
        mushu.drivetrainMode(Motor.RunMode.RawPower);
    }
    @Override
    public void execute(){

        m_driveSubsystem.driveFieldCentric(backward, strafe, turn, -heading);
        backward = mushu.driverGamepad.getLeftY();
        strafe = -mushu.driverGamepad.getLeftX();
        turn = -mushu.driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - -mushu.driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        heading =  mushu.getHeading();
        telemetry.addData("Theta",  mushu.getHeading());
        telemetry.addData("battery Voltage", mushu.getBatteryVoltage());
        telemetry.update();

    }

}
