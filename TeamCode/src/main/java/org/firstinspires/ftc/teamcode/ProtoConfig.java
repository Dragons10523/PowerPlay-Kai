package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    DigitalChannel limitLeft, limitRight;

    private boolean reverseMotors = true;

    final private static double P = 1.1294;
    final private static double I = 0.1129;
    final private static double D = 0;
    final private static double F = 11.2941;

    public ProtoConfig(HardwareMap hwmap){
        frontRight = hwmap.get(DcMotorEx.class, "frontRight");
        rearRight = hwmap.get(DcMotorEx.class, "rearRight");
        frontLeft = hwmap.get(DcMotorEx.class, "frontLeft");
        rearLeft = hwmap.get(DcMotorEx.class, "rearLeft");

        if(reverseMotors) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        try {
            left = hwmap.get(DistanceSensor.class, "leftLazer");
            right = hwmap.get(DistanceSensor.class, "rightLazer");
            rear = hwmap.get(DistanceSensor.class, "rearLazer");
            front = hwmap.get(DistanceSensor.class, "frontLazer");
            cameraDis = hwmap.get(DistanceSensor.class, "cameraLazer");
            limitLeft = hwmap.get(DigitalChannel.class, "limitLeft");
            limitRight = hwmap.get(DigitalChannel.class, "limitRight");
            limitLeft.setMode(DigitalChannel.Mode.INPUT);
            limitRight.setMode(DigitalChannel.Mode.INPUT);
        }catch (Exception e){
            Log.d("Config Error", e.getMessage());
        }

        lift = hwmap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blockIntake = hwmap.get(Servo.class, "blockIntake");
        intake1 = hwmap.get(CRServo.class, "intake1");
        intake2 = hwmap.get(CRServo.class, "intake2");

        horizontal = hwmap.get(DcMotorEx.class, "horizontal");
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motors = {frontRight, frontLeft, rearRight, rearLeft};

        for(DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            motor.setPositionPIDFCoefficients(5);
        }

        ramp1 = hwmap.get(DcMotorEx.class, "ramp1");
        ramp2 = hwmap.get(DcMotorEx.class, "ramp2");
        leftClaw = hwmap.get(Servo.class, "leftClaw");
        rightClaw = hwmap.get(Servo.class, "rightClaw");
        leftClaw.setDirection(Servo.Direction.REVERSE);
    }
}
