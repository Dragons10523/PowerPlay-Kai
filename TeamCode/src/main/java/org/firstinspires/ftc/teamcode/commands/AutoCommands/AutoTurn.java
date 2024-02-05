package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;



import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

public class AutoTurn extends CommandBase {
    Mushu mushu;
    int angle;
    MecanumDriveSubsystems sub;

    boolean isFinished = false;

    public AutoTurn(int angle, MecanumDriveSubsystems sub, Mushu mushu){

        this.angle = angle;
        this.mushu = mushu;
        this.sub = sub;

    }
    public void execute(){
        double angularDistance;
        double initialAngle = mushu.getHeading();


        do{
            double turnVal = 1;

            if(angle - initialAngle < 0) turnVal = -1;

            angularDistance = Math.abs(angle - initialAngle);
            if(angularDistance > 180){ // dealing with edge case
                turnVal *= -1;
                angularDistance = 360 - angularDistance;
            }
            double powerReduce = angularDistance / 40;

            powerReduce = Math.max(powerReduce, 0.2);
            powerReduce = Math.min(powerReduce, 1);

            sub.manualDrive(-turnVal * powerReduce, turnVal * powerReduce, -turnVal * powerReduce, turnVal * powerReduce);

        }
        while(angularDistance > 3);

        isFinished = true;
        this.cancel();
    }
    public boolean isFinished(){
        return isFinished;
    }

    public void end(boolean interrupted){
        mushu.mecanum.stop();
        mushu.stopMotors();
    }

}
