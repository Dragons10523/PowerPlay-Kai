package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Mushu;

public class DroneSub extends SubsystemBase {
    Mushu mushu;

    public DroneSub(Mushu mushu){
        this.mushu = mushu;
    }
    public void flipDrone(){
        mushu.droneServo.setPosition(1);
    }
}
