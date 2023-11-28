package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Mushu;


public class MecanumDriveSubsystems extends SubsystemBase{
    public final MecanumDrive drive;

    public MecanumDriveSubsystems(final Mushu mushu) {

        this.drive = mushu.mecanum;

    }

    public void driveFieldCentric(double forward, double strafe, double turn, double heading) {
        drive.driveFieldCentric(strafe, forward, turn, heading, false);
    }


}
