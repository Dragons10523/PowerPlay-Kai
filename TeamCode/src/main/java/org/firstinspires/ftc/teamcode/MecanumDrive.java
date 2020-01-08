package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrive {
    public DcMotorEx fl, fr, rr, rl;

    public MecanumDrive(DriveTrain dt){
        fl = dt.frontLeft;fr = dt.frontRight;rr = dt.rearRight;rl = dt.rearLeft;
    }

    public enum Relativity {
        ROBOT, FIELD
    }


    public void joystickMove(double leftY, double leftX, double rightX){
        double power = Math.hypot(leftX, leftY);
        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;
        fr.setPower(power * Math.cos(robotAngle) + rightX);
        rr.setPower(power * Math.sin(robotAngle) + rightX);
        fl.setPower(power * Math.sin(robotAngle) - rightX);
        rl.setPower(power * Math.cos(robotAngle) - rightX);

    }

    public void absMove(double angle, double power, double gyro){
        double a = Math.toRadians(angle)+ Math.PI/4 + Math.toRadians(gyro);
        fl.setPower(power * Math.sin(a));
        rl.setPower(power * Math.cos(a));
        fr.setPower(power * Math.cos(a));
        rr.setPower(power * Math.sin(a));
    }

    public void move(double angle, double power, double turn){
        double a = Math.toRadians(angle)- Math.PI/4;
        fl.setPower(power * Math.sin(a) - turn);
        rl.setPower(power * Math.cos(a) - turn);
        fr.setPower(power * Math.cos(a) + turn);
        rr.setPower(power * Math.sin(a) + turn);
    }

    public void absoluteMove(double leftY, double leftX, double angle, double rightX){
        double power  = Math.hypot(leftX, leftY);
        double robotAngle = Math.atan2(leftY, leftX) -(Math.PI/4) +angle;
        fr.setPower(power * Math.cos(robotAngle) + rightX);
        rr.setPower(power * Math.sin(robotAngle) + rightX);
        fl.setPower(power * Math.sin(robotAngle) - rightX);
        rl.setPower(power * Math.cos(robotAngle) - rightX);
    }

    public void stopNow(){
        fr.setPower(0);
        rr.setPower(0);
        fl.setPower(0);
        rl.setPower(0);
    }



}
