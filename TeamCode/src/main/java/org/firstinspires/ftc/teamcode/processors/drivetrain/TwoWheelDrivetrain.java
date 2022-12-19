package org.firstinspires.ftc.teamcode.processors.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TwoWheelDrivetrain extends AbstractDrivetrain {
    public TwoWheelDrivetrain(DcMotor left, DcMotor right) {
        super(left, right, left, right);
    }
}
