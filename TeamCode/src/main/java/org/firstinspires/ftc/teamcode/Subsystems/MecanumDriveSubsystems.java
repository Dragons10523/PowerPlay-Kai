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
    public void driveMecanum(double power, double angle, boolean interrupted){
        if(interrupted)
        {
            drive.stop();
        }
        else {


            double strafe;
            double drive;
            strafe = Math.cos(angle) - Math.sin(angle);
            drive = Math.sin(angle) + Math.cos(angle);
            double[] speeds = {
                    (drive + strafe),
                    (drive - strafe),
                    (drive - strafe),
                    (drive + strafe)
            };
            manualDrive(speeds[0], speeds[1], speeds[2], speeds[3]);
        }

    }
    public void driveWithEncoders(){

    }

    public void resetIMU(){
        mushu.stopMotors();
        mushu.resetIMU();
    }
}

