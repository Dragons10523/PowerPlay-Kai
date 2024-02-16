package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

public class WiggleClawDown extends CommandBase {
    Mushu mushu;

    MecanumDriveSubsystems sub;
    Telemetry telemetry;
    boolean isFinished = false;
    public WiggleClawDown(Mushu mushu, MecanumDriveSubsystems sub, Telemetry telemetry){
        this.mushu = mushu;
        this.sub = sub;
        this.telemetry = telemetry;

    }
    public void execute(){
        sub.manualDrive(1,-1,1,-1);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sub.manualDrive(-1,1,-1,1);

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sub.manualDrive(0,0,0,0);

        this.cancel();
        isFinished = true;
    }
    public void end(boolean interrupted){
        mushu.mecanum.stop();
        mushu.intakeArm.set(0);
    }
    public boolean isFinished(){
        return isFinished;
    }
}
