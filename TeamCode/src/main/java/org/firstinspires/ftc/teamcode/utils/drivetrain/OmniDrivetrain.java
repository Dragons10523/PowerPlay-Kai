package org.firstinspires.ftc.teamcode.utils.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.VecUtils;

public class OmniDrivetrain extends AbstractOmniDrivetrain {
    public OmniDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight, 0);
    }
}
