package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class HardwareConfig {
    //All Hardware Devices
    public DcMotorEx frontRight, frontLeft, rearRight, rearLeft, ramp1, ramp2, lift, horizontal;
    public DistanceSensor[] distanceSensors;
    public Servo leftClaw, rightClaw, blockIntake, leftHook, rightHook, capStone;
    public CRServo intake1, intake2;
    public DigitalChannel limitLeft;
    public BNO055IMU imu;
    public DistanceSensor cameraDis;

    HardwareMap HWMAP;

    final private static double P = 1.1294;
    final private static double I = 0.1129;
    final private static double D = 0;
    final private static double F = 11.2941;

    public HardwareConfig(HardwareMap hwmap){
        HWMAP = hwmap;
    }

    public void initializeDriveTrain(){
        boolean reverseMotors = true;

        frontRight =    HWMAP.get(DcMotorEx.class, "frontRight");
        rearRight =     HWMAP.get(DcMotorEx.class, "rearRight");
        frontLeft =     HWMAP.get(DcMotorEx.class, "frontLeft");
        rearLeft =      HWMAP.get(DcMotorEx.class, "rearLeft");

        DcMotorEx[] motors = {frontRight, frontLeft, rearRight, rearLeft};
        for(DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            motor.setPositionPIDFCoefficients(5);
        }
        if(reverseMotors) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void initializeDistanceSensors() {
        distanceSensors = new DistanceSensor[4];
        distanceSensors[0] =    HWMAP.get(DistanceSensor.class, "leftLazer");
        distanceSensors[1] =    HWMAP.get(DistanceSensor.class, "rightLazer");
        distanceSensors[2] =    HWMAP.get(DistanceSensor.class, "rearLazer");
        distanceSensors[3] =    HWMAP.get(DistanceSensor.class, "frontLazer");
    }
    public void initializeCameraDistance(){
        cameraDis = HWMAP.get(DistanceSensor.class, "cameraDis");
    }

    public void initializeLimitSwitches(){
        limitLeft =     HWMAP.get(DigitalChannel.class, "limitLeft");
        limitLeft.setMode(DigitalChannel.Mode.INPUT);
    }

    public void initializeTools(){
        lift =      HWMAP.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blockIntake =   HWMAP.get(Servo.class, "blockIntake");
        intake1 =       HWMAP.get(CRServo.class, "intake1");
        intake2 =       HWMAP.get(CRServo.class, "intake2");

        horizontal =    HWMAP.get(DcMotorEx.class, "horizontal");
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        ramp1 =         HWMAP.get(DcMotorEx.class, "ramp1");
        ramp2 =         HWMAP.get(DcMotorEx.class, "ramp2");

        capStone = HWMAP.get(Servo.class, "capstone");
    }

    public void initializeServos(){
        leftClaw =      HWMAP.get(Servo.class, "leftClaw");
        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw =     HWMAP.get(Servo.class, "rightClaw");
        leftHook = HWMAP.get(Servo.class, "leftHook");
        rightHook = HWMAP.get(Servo.class, "rightHook");
    }

    public void initializeIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = HWMAP.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds()<100){}
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        time.reset();
        while(time.milliseconds()<100){}
    }
}
