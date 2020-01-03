package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DriveTrain {
    public DcMotorEx frontLeft, frontRight, rearLeft, rearRight;

    public DriveTrain(DcMotorEx fl, DcMotorEx fr, DcMotorEx rl, DcMotorEx rr){
        frontLeft   = fl;
        frontRight  = fr;
        rearLeft    = rl;
        rearRight   = rr;
    }
}
