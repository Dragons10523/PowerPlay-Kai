package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    public Limelight3A limelight;
    public final IMU imu;
    public SparkFunOTOS opticalSensor;
    public VoltageSensor voltageSensor;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    HardwareMap hwmap;
    public static enum SERVOS{
        ARM_LEFT,
        ARM_RIGHT,
        BUCKET,
        INTAKE_SERVO,
        SPECIMEN_GRABBER,
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
        ARM_FLIP,
        LIFT
    }
    public RobotClass(HardwareMap hwmap) {
        this.hwmap = hwmap;

        opticalSensor = hwmap.get(SparkFunOTOS.class, "opticalSensor");
        //I2C port 1 control
        //colorSensor = hwmap.get(ColorSensor.class, "colorSensor");

        Motors = new HashMap<>();
        Servos = new HashMap<>();
        CR_Servos = new HashMap<>();
        Motors.put(MOTORS.FRONT_LEFT, hwmap.get(DcMotorEx.class, "frontLeft"));
        //port 0 control
        Motors.put(MOTORS.FRONT_RIGHT, hwmap.get(DcMotorEx.class, "frontRight"));
        //port 3 Expansion
        Motors.put(MOTORS.BACK_LEFT, hwmap.get(DcMotorEx.class, "backLeft"));
        //port 2 control
        Motors.put(MOTORS.BACK_RIGHT, hwmap.get(DcMotorEx.class, "backRight"));
        //port 1 control
        voltageSensor = hwmap.get(VoltageSensor.class, "Control Hub");
        imu = hwmap.get(IMU.class, "imu");
        //initMotorsProto();
        initMotorsComp();
        new Utils(this);
        drivetrain = new MecanumDrive(Motors, this);
        limelight = hwmap.get(Limelight3A.class, "limelight");

//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP);
//
//        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //IMU becomes defective when the voltage drops too low
    }
    public double getHeading() {
        return opticalSensor.getPosition().h;
    }
    public void resetIMU() {
        SparkFunOTOS.Pose2D pos = opticalSensor.getPosition();
        SparkFunOTOS.Pose2D posNew = new SparkFunOTOS.Pose2D(pos.x, pos.y, 0);

        opticalSensor.setPosition(posNew);
    }
    private void initMotorsProto() {
        Motors.get(MOTORS.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.get(MOTORS.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.get(MOTORS.FRONT_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);
        Motors.get(MOTORS.BACK_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);

        //Proto-Bot
    }
    public void initMotorsComp(){
        colorSensor = hwmap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hwmap.get(DistanceSensor.class, "sensor_color_distance");

        Motors.put(MOTORS.LIFT_LEFT, hwmap.get(DcMotorEx.class, "liftLeft"));
        //port 0 Expansion
        Motors.put(MOTORS.LIFT_RIGHT, hwmap.get(DcMotorEx.class,"liftRight"));
        //port 1 Expansion
        Motors.put(MOTORS.LIFT, hwmap.get(DcMotorEx.class, "lift"));
        //RevSparkMini servo port 0 Control
        Motors.put(MOTORS.ARM_FLIP, hwmap.get(DcMotorEx.class, "armFlip"));
        //port 2 Expansion
        Servos.put(SERVOS.ARM_LEFT, hwmap.get(Servo.class, "armLeft"));
        //port 0 expansion
        Servos.put(SERVOS.ARM_RIGHT, hwmap.get(Servo.class, "armRight"));
        //port 1 expansion
        Servos.put(SERVOS.BUCKET, hwmap.get(Servo.class, "bucket"));
        //port 2 expansion
        Servos.put(SERVOS.INTAKE_SERVO, hwmap.get(Servo.class, "intakeServo"));
        //port 3 expansion
        Servos.put(SERVOS.SPECIMEN_GRABBER, hwmap.get(Servo.class, "specimenGrabber"));
        //port 4 expansion
        CR_Servos.put(CR_SERVOS.INTAKE, hwmap.get(CRServo.class, "intake"));
        //port 5 expansion

        Motors.get(MOTORS.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.get(MOTORS.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.get(MOTORS.FRONT_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);
        Motors.get(MOTORS.BACK_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);

        Motors.get(MOTORS.FRONT_LEFT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors.get(MOTORS.BACK_LEFT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors.get(MOTORS.FRONT_RIGHT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors.get(MOTORS.BACK_RIGHT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Motors.get(MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Motors.get(MOTORS.ARM_FLIP).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void initResetLift(){
        Motors.get(MOTORS.LIFT).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
