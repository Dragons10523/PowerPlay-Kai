package org.firstinspires.ftc.teamcode.utils.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FourWheelDrivetrain extends AbstractDrivetrain {
    public FourWheelDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight);
    }
}
