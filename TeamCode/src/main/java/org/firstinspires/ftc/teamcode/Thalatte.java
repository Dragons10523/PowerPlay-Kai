package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Thalatte {
    public DcMotor frontLeft, frontRight, backLeft, backRight, shooterBack, shooterFront, intake1, intake2, vwomp;
    public Servo vwompClampLeft, vwompClampRight;
    //public CRServo intake1, intake2;
    public BNO055IMU imu;

    public HardwareMap hwmap;
    public MouseTrap mouse;

    public Geometry geometry;
    public Lightsaber lightsaber;

    public Thalatte(HardwareMap hwmap) {
        this.hwmap = hwmap;

        frontLeft  = hwmap.get(DcMotor.class,"frontLeft");
        frontRight = hwmap.get(DcMotor.class,"frontRight");
        backLeft   = hwmap.get(DcMotor.class,"backLeft");
        backRight  = hwmap.get(DcMotor.class,"backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        shooterBack  = hwmap.get(DcMotor.class, "shooterBack");
//        shooterFront = hwmap.get(DcMotor.class, "shooterFront");

        intake1 = hwmap.get(DcMotor.class, "intake1");
        intake2 = hwmap.get(DcMotor.class, "intake2");

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        vwomp  = hwmap.get(DcMotor.class, "vwomp");
//
//        vwompClampLeft  = hwmap.get(Servo.class, "vwompClampLeft");
//        vwompClampRight = hwmap.get(Servo.class, "vwompClampRight");

        //intake1 = hwmap.get(CRServo.class, "intake1");
        //intake2 = hwmap.get(CRServo.class, "intake2");

        BNO055IMU.Parameters parameters             = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled                   = true;
        parameters.loggingTag                       = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //mouse = new MouseTrap(hwmap.appContext);

        geometry = new Geometry();

        HashMap<String, Lightsaber.LightsaberUnit> lightsaberMap = new HashMap<>();
        lightsaberMap.put("rightSaber", new Lightsaber.LightsaberUnit(0,0,0)            );
        lightsaberMap.put("leftSaber" , new Lightsaber.LightsaberUnit(0,0,Math.PI)      );
        lightsaberMap.put("frontSaber", new Lightsaber.LightsaberUnit(0,0,Math.PI/2)    );
        lightsaberMap.put("backSaber" , new Lightsaber.LightsaberUnit(0,0,Math.PI * 1.5));

        //lightsaber = new Lightsaber(lightsaberMap,hwmap,geometry);
    }
}