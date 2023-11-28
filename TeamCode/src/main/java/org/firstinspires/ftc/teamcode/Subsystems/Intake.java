package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Mushu;

public class Intake extends Subsystem {
    Mushu mushu;
    public Intake(Mushu mushu){
        this.mushu = mushu;
    }
    public void run(){
        mushu.intakeServo.set(1);
    }
    public void stop(){
        mushu.intakeServo.set(0);
    }
}
