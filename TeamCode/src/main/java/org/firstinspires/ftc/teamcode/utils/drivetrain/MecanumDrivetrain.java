package org.firstinspires.ftc.teamcode.utils.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.VecUtils;

public class MecanumDrivetrain extends AbstractOmniDrivetrain {
    public MecanumDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight, VecUtils.HALF_PI/2);
    }
}
