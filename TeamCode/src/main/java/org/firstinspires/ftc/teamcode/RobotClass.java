package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.AbstractOmniDrivetrain;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;
import java.util.Map;

public class RobotClass {
    public AbstractOmniDrivetrain drivetrain;

    public Map<MOTORS, DcMotorEx> Motors;
    public Map<SERVOS, Servo> Servos;
    public Map<CR_SERVOS, CRServo> CR_Servos;
    public final IMU imu;
    public OpenCvWebcam camera1;
    public WebcamName webcamName;
    public Utils utils;

    public SparkFunOTOS opticalSensor;
    public VoltageSensor voltageSensor;
    HardwareMap hwmap;
    public static enum SERVOS{
        ARM_LEFT,
        ARM_RIGHT,
        BUCKET,
    }
    public static enum CR_SERVOS{
        INTAKE,
    }
    public static enum MOTORS {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        LIFT_LEFT,
        LIFT_RIGHT,
        ARM_FLIP
    }

    public RobotClass(HardwareMap hwmap) {
        this.hwmap = hwmap;

        imu = hwmap.get(IMU.class, "imu");
        Motors = new HashMap<>();
        Servos = new HashMap<>();
        CR_Servos = new HashMap<>();
        Motors.put(MOTORS.FRONT_LEFT, hwmap.get(DcMotorEx.class, "frontLeft"));
        Motors.put(MOTORS.FRONT_RIGHT, hwmap.get(DcMotorEx.class, "frontRight"));
        Motors.put(MOTORS.BACK_LEFT, hwmap.get(DcMotorEx.class, "backLeft"));
        Motors.put(MOTORS.BACK_RIGHT, hwmap.get(DcMotorEx.class, "backRight"));
        voltageSensor = hwmap.get(VoltageSensor.class, "Control Hub");



//        Motors.put(MOTORS.LIFT_LEFT, hwmap.get(DcMotorEx.class, "liftLeft"));
//        Motors.put(MOTORS.LIFT_RIGHT, hwmap.get(DcMotorEx.class,"liftRight"));
//        Motors.put(MOTORS.ARM_FLIP, hwmap.get(DcMotorEx.class, "armFlip"));
//        //port 1, 2, 3 expansion hub
//
//        Servos.put(SERVOS.ARM_LEFT, hwmap.get(Servo.class, "armLeft"));
//        Servos.put(SERVOS.ARM_RIGHT, hwmap.get(Servo.class, "armRight"));
//        Servos.put(SERVOS.BUCKET, hwmap.get(Servo.class, "bucket"));
//        //port 1, 2 expansion hub
//
//        CR_Servos.put(CR_SERVOS.INTAKE, hwmap.get(CRServo.class, "intake"));
        //port 5 expansion hub
        Utils utils = new Utils(this);
        drivetrain = new MecanumDrive(Motors, this);

        opticalSensor = hwmap.get(SparkFunOTOS.class, "opticalSensor");
        //webcamName = hwmap.get(WebcamName.class, "Webcam 1");
        //camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    public double getHeading() {
        return opticalSensor.getPosition().h;
    }
    public void resetIMU() {
        opticalSensor.calibrateImu(100, true);
    }
    public void initMotorsProto() {
        Motors.get(MOTORS.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.get(MOTORS.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.get(MOTORS.FRONT_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);
        Motors.get(MOTORS.BACK_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);

        //Proto-Bot
    }
    public void initMotorsComp(){
        Motors.get(MOTORS.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.get(MOTORS.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.get(MOTORS.FRONT_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);
        Motors.get(MOTORS.BACK_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);

        Motors.get(MOTORS.FRONT_RIGHT).setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Motors.get(MOTORS.FRONT_LEFT).setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Motors.get(MOTORS.BACK_LEFT).setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Motors.get(MOTORS.BACK_RIGHT).setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Motors.get(MOTORS.LIFT_LEFT).setDirection(DcMotorSimple.Direction.FORWARD);
        Motors.get(MOTORS.LIFT_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);
        Motors.get(MOTORS.LIFT_LEFT).setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Motors.get(MOTORS.LIFT_RIGHT).setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }


}
