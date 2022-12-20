package org.firstinspires.ftc.teamcode.processors.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.processors.VecUtils;

public class OmniDrivetrain extends AbstractOmniDrivetrain {
    public OmniDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight, -VecUtils.HALF_PI/2);
    }
}
