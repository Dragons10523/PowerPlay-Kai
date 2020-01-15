package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoders{
    public static final double COUNTS_ROTATION = 560;
    public static final double WHEEL_DIAMETER = 3.0;
    public DcMotorEx frontRight, frontLeft, rearRight, rearLeft;


    public Encoders(DriveTrain dt){
        frontRight = dt.frontRight;
        frontLeft = dt.frontLeft;
        rearRight = dt.rearRight;
        rearLeft = dt.rearLeft;

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders(){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getInches(){
        double frontRightInch = Math.abs((Math.PI)*WHEEL_DIAMETER*(frontRight.getCurrentPosition()/COUNTS_ROTATION));
        double rearRightInch = Math.abs((Math.PI)*WHEEL_DIAMETER*(rearRight.getCurrentPosition()/COUNTS_ROTATION));
        double frontLeftInch = Math.abs((Math.PI)*WHEEL_DIAMETER*(frontLeft.getCurrentPosition()/COUNTS_ROTATION));
        double rearLeftInch = Math.abs((Math.PI)*WHEEL_DIAMETER*(rearLeft.getCurrentPosition()/COUNTS_ROTATION));

        return (frontLeftInch + rearLeftInch + frontRightInch + rearRightInch)/4;
    }

    public double getTicks(){

        double fr = Math.abs(frontRight.getCurrentPosition());
        double rr = Math.abs(rearRight.getCurrentPosition());
        double fl = Math.abs(frontLeft.getCurrentPosition());
        double rl = Math.abs(rearLeft.getCurrentPosition());

        return (fl + rl + fr + rr)/4;
    }
}
