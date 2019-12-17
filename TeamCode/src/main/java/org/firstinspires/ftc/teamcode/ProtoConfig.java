package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class ProtoConfig {
    public DcMotorEx frontRight, frontLeft, rearRight, rearLeft, ramp1, ramp2, lift, horizontal;
    public DistanceSensor left, right, rear, front, cameraDis;
    public Servo leftClaw, rightClaw, blockIntake;
    public CRServo intake1, intake2;

    private DcMotorEx[] motors = {null, null, null, null};
    final private static double P = 1.1294;
    final private static double I = 0.1129;
    final private static double D = 0;
    final private static double F = 11.2941;
    public void init(HardwareMap hwmap){
        frontRight = hwmap.get(DcMotorEx.class, "frontRight");
        rearRight = hwmap.get(DcMotorEx.class, "rearRight");
        frontLeft = hwmap.get(DcMotorEx.class, "frontLeft");
        rearLeft = hwmap.get(DcMotorEx.class, "rearLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        try {
            left = hwmap.get(DistanceSensor.class, "leftLazer");
            right = hwmap.get(DistanceSensor.class, "rightLazer");
            rear = hwmap.get(DistanceSensor.class, "rearLazer");
            front = hwmap.get(DistanceSensor.class, "frontLazer");
            cameraDis = hwmap.get(DistanceSensor.class, "cameraLazer");
        }catch (Exception e){
            Log.d("Config Error", e.getMessage());
        }
        try {
            lift = hwmap.get(DcMotorEx.class, "lift");
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            blockIntake = hwmap.get(Servo.class, "blockIntake");
            intake1 = hwmap.get(CRServo.class, "intake1");
            intake2 = hwmap.get(CRServo.class, "intake2");
            horizontal = hwmap.get(DcMotorEx.class, "horizontal");
            horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception e){
            Log.d("Config Error", "Wrong Robot");
        }

        motors[0] = frontRight;
        motors[1] = frontLeft;
        motors[2] = rearRight;
        motors[3] = rearLeft;

        for(int i = 0; i < motors.length; i++){
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            motors[i].setPositionPIDFCoefficients(5);
        }
        ramp1 = hwmap.get(DcMotorEx.class, "ramp1");
        ramp2 = hwmap.get(DcMotorEx.class, "ramp2");
        leftClaw = hwmap.get(Servo.class, "leftClaw");
        rightClaw = hwmap.get(Servo.class, "rightClaw");
        leftClaw.setDirection(Servo.Direction.REVERSE);
    }
}
