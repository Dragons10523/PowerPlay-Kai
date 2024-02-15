package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Mushu;

public class MotorDisable extends SubsystemBase {
    Mushu mushu;
    public MotorDisable(Mushu mushu){
        this.mushu = mushu;
    }
    public void disableHang(){
        mushu.hangMotor.disable();
    }
    public void enableHang(){
        mushu.hangMotor.disable();
    }
}
