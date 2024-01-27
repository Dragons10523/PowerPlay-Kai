package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Mushu;

public class InExtakeSub extends SubsystemBase {
    Mushu mushu;
    public InExtakeSub(Mushu mushu){
        this.mushu = mushu;
    }
    public void runIN(double power){
        mushu.intakeMotor.set(-power);
    }
    public void runEX(double power){
        mushu.extakeServo.set(power);
    }
    public void flipGate(){mushu.gateServo.setPosition(1);}
    public void retractGate(){mushu.gateServo.setPosition(0);}

}
