package org.firstinspires.ftc.teamcode.utils.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class AbstractDrivetrain {
    DcMotor[] driveMotors;

    public AbstractDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
    }

    public void drive(double forwards, double turn) {
        driveMotors[0].setPower(forwards + turn);
        driveMotors[1].setPower(forwards - turn);
        driveMotors[2].setPower(forwards + turn);
        driveMotors[3].setPower(forwards - turn);
    }
}
