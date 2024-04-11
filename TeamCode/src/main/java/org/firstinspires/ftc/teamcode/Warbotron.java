package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Warbotron {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor hammer1;
    public DcMotor hammer2;
    public CRServo flyWheel;
    public Servo spatula;

    public Warbotron(HardwareMap hwmap){
        frontLeft = hwmap.get(DcMotor.class, "FrontLeft");
        frontRight = hwmap.get(DcMotor.class, "FrontRight");
        backLeft = hwmap.get(DcMotor.class, "BackLeft");
        backRight = hwmap.get(DcMotor.class, "BackRight");
        hammer1 = hwmap.get(DcMotor.class, "Hammer1");
        hammer2 = hwmap.get(DcMotor.class, "Hammer2");
        flyWheel = hwmap.get(CRServo.class, "flyWheel");
        spatula = hwmap.get(Servo.class, "spatula");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hammer1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hammer2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        hammer1.setDirection(DcMotorSimple.Direction.FORWARD);
        hammer2.setDirection(DcMotorSimple.Direction.FORWARD);

        hammer1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hammer2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
