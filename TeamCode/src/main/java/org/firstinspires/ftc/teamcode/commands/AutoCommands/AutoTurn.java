package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

public class AutoTurn extends CommandBase {
    Mushu mushu;
    int angle;
    MecanumDriveSubsystems sub;
    double loopCounter = 0;

    boolean isFinished = false;
    static double angularDistance = 0;
    Telemetry telemetry;

    public AutoTurn(int angle, MecanumDriveSubsystems sub, Mushu mushu, Telemetry telemetry){

        this.angle = angle;
        this.mushu = mushu;
        this.sub = sub;
        this.telemetry = telemetry;

    }
    public void execute(){
        isFinished = false;
        double initialAngle = mushu.getHeading();
        mushu.drivetrainMode(Motor.RunMode.RawPower);
        loopCounter++;



        do{
            double turnVal = 1;


            if(angle - initialAngle < 0) turnVal = -1; // checks which way it should turn

            double currentAngle = mushu.getHeading();

            angularDistance = Math.abs(currentAngle - angle);
            if(angularDistance > 180){ // dealing with edge case
                turnVal *= -1;
                angularDistance = 360 - angularDistance;
            }
            double powerReduce = angularDistance / 90;

            powerReduce = Math.max(powerReduce, 0.2);
            powerReduce = Math.min(powerReduce, 1);

            sub.manualDrive(turnVal * powerReduce, turnVal * powerReduce, turnVal * powerReduce, turnVal * powerReduce);
            telemetry.addData("turnVal", turnVal);
            telemetry.addData("angularDistance", angularDistance);
            telemetry.addData("initialAngle", initialAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("powerReduce", powerReduce);
            telemetry.addData("power", turnVal * powerReduce);
            telemetry.addData("atTarget", atTarget());
            telemetry.addData("loopCounter", loopCounter);


            telemetry.update();
        }
        while(!atTarget());


        this.cancel();


    }
    public void end(boolean interrupted){
        mushu.mecanum.stop();
        mushu.stopMotors();
        isFinished = true;
    }

    public static boolean atTarget(){
        return angularDistance < 1.5;
    }

    public boolean isFinished(){
        return atTarget();
    }

}
