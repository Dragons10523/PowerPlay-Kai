package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.DriveAuto;


public class MecanumDriveSubsystems extends SubsystemBase{
    public final MecanumDrive drive;

    public MecanumDriveSubsystems(final Mushu mushu) {

        this.drive = mushu.mecanum;

    }

    public void driveFieldCentric(double forward, double strafe, double turn, double heading) {
        drive.driveFieldCentric(strafe, forward, turn, heading, false);
    }
    public void driveFieldAuto(DriveAuto.TeamColor color){
        if(color == DriveAuto.TeamColor.BLUE){

        }

    }

}
