package org.firstinspires.ftc.teamcode.processors.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class AbstractDrivetrain {
    public DcMotor[] driveMotors;

    public AbstractDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
    }

    public void setDriveDirection(DcMotorSimple.Direction direction) {
        switch(direction) {
            case REVERSE:
                driveMotors[0].setDirection(DcMotorSimple.Direction.FORWARD);
                driveMotors[2].setDirection(DcMotorSimple.Direction.FORWARD);
                driveMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
                driveMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
            case FORWARD:
            default:
                driveMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
                driveMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
                driveMotors[1].setDirection(DcMotorSimple.Direction.FORWARD);
                driveMotors[3].setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void drive(double forwards, double turn) {
        driveMotors[0].setPower(forwards + turn);
        driveMotors[1].setPower(forwards - turn);
        driveMotors[2].setPower(forwards + turn);
        driveMotors[3].setPower(forwards - turn);
    }

    public void setMode(DcMotor.RunMode mode) {
        for(DcMotor motor : driveMotors) {
            motor.setMode(mode);
        }
    }

    public void setZeroBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for(DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }
}
