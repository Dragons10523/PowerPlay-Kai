package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

public class WiggleClawDown extends CommandBase {
    Mushu mushu;
    CommandScheduler commandScheduler;
    MecanumDriveSubsystems sub;
    public WiggleClawDown(Mushu mushu, CommandScheduler commandScheduler, MecanumDriveSubsystems sub){
        this.mushu = mushu;
        this.commandScheduler = commandScheduler;
        this.sub = sub;

    }
    public void execute(){
        commandScheduler.schedule(new AutoDrive(1,1,0, sub, mushu));
        mushu.intakeArm.set(1);

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        commandScheduler.schedule(new AutoDrive(1,-1, 0, sub,mushu));
        mushu.intakeArm.set(-1);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        mushu.intakeArm.set(0);

        this.cancel();
    }
    public void end(boolean interrupted){
        mushu.mecanum.stop();
        mushu.intakeArm.set(0);
    }
}
