package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.HashMap;
import java.util.Iterator;

public class Thalatte {
    public DcMotor frontLeft, frontRight, backLeft, backRight, shooterBack, shooterFront, intake, vwomp;
    public Servo vwompClamp;
    public CRServo feeder;
    public BNO055IMU imu;

    public HardwareMap hwmap;

    public DistanceSensor front, right, left, back;

    public VoltageSensor vs = null;

//    public Blinker lights;

    public WebcamName webcam;

    public Thalatte(HardwareMap hwmap) {
        this.hwmap = hwmap;

//        lights = hwmap.get(Blinker.class, "lights");

        frontLeft  = hwmap.get(DcMotor.class,"frontLeft");
        frontRight = hwmap.get(DcMotor.class,"frontRight");
        backLeft   = hwmap.get(DcMotor.class,"backLeft");
        backRight  = hwmap.get(DcMotor.class,"backRight");
//
//        frontLeft .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight .setDirection(DcMotor.Direction.REVERSE);
        frontLeft .setDirection(DcMotor.Direction.FORWARD);
        backLeft  .setDirection(DcMotor.Direction.FORWARD);
//
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterBack  = hwmap.get(DcMotor.class, "shooterBack");
        shooterFront = hwmap.get(DcMotor.class, "shooterFront");
        shooterBack.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hwmap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        vwomp  = hwmap.get(DcMotor.class, "vwomp");
        vwomp.setDirection(DcMotorSimple.Direction.REVERSE);
        vwomp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vwomp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vwompClamp  = hwmap.get(Servo.class, "vwompClamp");

        feeder = hwmap.get(CRServo.class, "feeder");

        feeder.setDirection(DcMotorSimple.Direction.FORWARD);

        BNO055IMU.Parameters parameters             = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled                   = true;
        parameters.loggingTag                       = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        front = hwmap.get(DistanceSensor.class, "frontSaber");
        right = hwmap.get(DistanceSensor.class, "rightSaber");
        left = hwmap.get(DistanceSensor.class, "leftSaber");
        back = hwmap.get(DistanceSensor.class, "backSaber");

        for(VoltageSensor s : hwmap.voltageSensor){
            vs = s;
            break;
        }

        webcam = hwmap.get(WebcamName.class, "Webcam 1");
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭