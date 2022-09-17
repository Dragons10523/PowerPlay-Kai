package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class Kai {
    // Also used for deadwheels. Use front motors for both y axis, back left for the x axis
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public DcMotor turntable, armLiftA, armLiftB;
    public Servo claw, clawFlup, clawTwist;
    public CRServo extendScrew, extendPinion;

    public BNO055IMU imu;

    public OpenCvWebcam frontCamera;

    public HardwareMap hwmap;

    public Kai(HardwareMap hwmap) {
        this.hwmap = hwmap;

        // Map electronics
        frontLeft = hwmap.get(DcMotor.class, "frontLeft");
        frontRight = hwmap.get(DcMotor.class, "frontRight");
        backLeft = hwmap.get(DcMotor.class, "backLeft");
        backRight = hwmap.get(DcMotor.class, "backRight");

        turntable = hwmap.get(DcMotor.class, "turntable");
        armLiftA = hwmap.get(DcMotor.class, "armLiftA");
        armLiftB = hwmap.get(DcMotor.class, "armLiftB");

        claw = hwmap.get(Servo.class, "claw");
        clawFlup = hwmap.get(Servo.class, "clawFlup");
        clawTwist = hwmap.get(Servo.class, "clawTwist");

        extendScrew = hwmap.get(CRServo.class, "extendScrew");
        extendPinion = hwmap.get(CRServo.class, "extendPinion");

        // Set up IMU parameters
        BNO055IMU.Parameters parameters             = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled                   = false;
        parameters.loggingTag                       = "IMU";

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        WebcamName frontWebcamName = hwmap.get(WebcamName.class, "Front Camera");
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(frontWebcamName);

        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up encoders
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setTargetPosition(0);
        armLiftA.setTargetPosition(0);
        armLiftB.setTargetPosition(0);
        turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turntable.setPower(1);
        armLiftA.setPower(1);
        armLiftB.setPower(1);
    }

    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS).firstAngle;
    }
}
