package org.firstinspires.ftc.teamcode.processors.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OmniDrivetrain extends AbstractOmniDrivetrain {
    public OmniDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight, 0);
    }
}
