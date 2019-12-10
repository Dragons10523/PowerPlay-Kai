package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class ProtoConfig {
    public DcMotor frontRight, frontLeft, rearRight, rearLeft, ramp1, ramp2, lift, horizontal;
    public DistanceSensor left, right, rear, front, cameraDis;
    public Servo leftClaw, rightClaw, blockIntake;
    public CRServo intake1, intake2;

    public void init(HardwareMap hwmap){
        frontRight = hwmap.dcMotor.get("frontRight");
        rearRight = hwmap.dcMotor.get("rearRight");
        frontLeft = hwmap.dcMotor.get("frontLeft");
        rearLeft = hwmap.dcMotor.get("rearLeft");

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
            lift = hwmap.get(DcMotor.class, "lift");
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            blockIntake = hwmap.get(Servo.class, "blockIntake");
            intake1 = hwmap.get(CRServo.class, "intake1");
            intake2 = hwmap.get(CRServo.class, "intake2");
            horizontal = hwmap.get(DcMotor.class, "horizontal");
            horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception e){
            Log.d("Config Error", "Wrong Robot");
        }

        ramp1 = hwmap.get(DcMotor.class, "ramp1");
        ramp2 = hwmap.get(DcMotor.class, "ramp2");
        leftClaw = hwmap.get(Servo.class, "leftClaw");
        rightClaw = hwmap.get(Servo.class, "rightClaw");
        leftClaw.setDirection(Servo.Direction.REVERSE);
    }
}
