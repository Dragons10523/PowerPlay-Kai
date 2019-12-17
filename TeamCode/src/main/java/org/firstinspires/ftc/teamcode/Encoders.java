package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoders{
    public static final double COUNTS_ROTATION = 560;
    public static final double WHEEL_DIAMETER = 3.0;
    public DcMotorEx frontRight, frontLeft, rearRight, rearLeft;

    public Encoders(DcMotorEx fr, DcMotorEx fl, DcMotorEx rr, DcMotorEx rl){
        frontRight = fr;
        frontLeft = fl;
        rearRight = rr;
        rearLeft = rl;

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
        double rearRightInch = Math.abs((Math.PI)*WHEEL_DIAMETER*(frontRight.getCurrentPosition()/COUNTS_ROTATION));
        double frontLeftInch = Math.abs((Math.PI)*WHEEL_DIAMETER*(frontRight.getCurrentPosition()/COUNTS_ROTATION));
        double rearLeftInch = Math.abs((Math.PI)*WHEEL_DIAMETER*(frontRight.getCurrentPosition()/COUNTS_ROTATION));

        return (frontLeftInch + rearLeftInch + frontRightInch + rearRightInch)/4;
    }

    public double getTicks(){

        return Math.abs(frontLeft.getCurrentPosition());
    }
}
