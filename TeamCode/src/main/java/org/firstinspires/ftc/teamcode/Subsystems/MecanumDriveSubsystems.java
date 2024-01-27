package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mushu;


public class MecanumDriveSubsystems extends SubsystemBase{
    public final MecanumDrive drive;
    Mushu mushu;

    public MecanumDriveSubsystems(final Mushu mushu) {

        this.drive = mushu.mecanum;
        this.mushu = mushu;
    }

    public void driveFieldCentric(double forward, double strafe, double turn, double heading) {
        drive.driveFieldCentric(strafe, forward, turn, heading, false);
    }
    public void manualDrive(double botLeft, double botRight, double topLeft, double topRight){
        drive.driveWithMotorPowers(topLeft, topRight, botLeft, botRight);
    }

    public void resetIMU(){
        mushu.stopMotors();
        mushu.resetIMU();
    }
}

