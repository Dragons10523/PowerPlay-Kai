package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

public class AutoTurn extends CommandBase {
    Mushu mushu;
    Double angle;
    MecanumDriveSubsystems sub;

    boolean isFinished = false;

    public AutoTurn(Mushu mushu, Double angle, MecanumDriveSubsystems sub){
        this.mushu = mushu;
        this.angle = angle;
        this.sub = sub;
    }
    public void initialize(){
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0,0,0);
    }
    public void execute(){
        double angularDistance = 0;
        double initialAngle = mushu.getHeading();
        double powerReduce = 1;

        do{
            double turnVal = 1;

            if(angle - initialAngle < 0) turnVal = -1;

            angularDistance = Math.abs(angle = initialAngle);
            if(angularDistance > 180){
                turnVal *= -1;
                angularDistance = 360 - angularDistance;
            }
            powerReduce = angularDistance / 40;

            powerReduce = Math.max(powerReduce, 0.2);
            powerReduce = Math.min(powerReduce, 1);

            sub.manualDrive(-turnVal * powerReduce, turnVal * powerReduce, -turnVal * powerReduce, turnVal * powerReduce);

        }
        while(angularDistance > 3);

        isFinished = true;
    }
    public boolean isFinished(){
        return isFinished;
    }

}
