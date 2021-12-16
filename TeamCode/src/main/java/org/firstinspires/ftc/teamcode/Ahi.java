package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

/* CLASS SUMMARY:
* Initializes all hardware and maps it from the hardware map to class variables
* Sets default values and modes where applicable
* */

public class Ahi {
    // Encoder goes in the motor port, A motors for drivetrain and arm motor for manipulation (CH = Control Hub | DH = Driver Hub)
    public DcMotor rightA, rightB, leftA, leftB; // Drivetrain (CH 0-3)
    public DcMotor arm, succc, capLift; // Manipulation Motors (DH 0-2)
    public CRServo ddr; // Manipulation Servos (CH 0)
    public Servo flup; // (CH 1)
    public BNO055IMU imu;

    public OpenCvCamera camera;

    int cameraMonitorViewId;

    public HardwareMap hwmap;

    final boolean drivetrainReverse = false;

    public Ahi(HardwareMap hwmap) {
        this.hwmap = hwmap;

        /*rightA = hwmap.get(DcMotor.class, "rightA");
        rightB = hwmap.get(DcMotor.class, "rightB");
        leftA = hwmap.get(DcMotor.class, "leftA");
        leftB = hwmap.get(DcMotor.class, "leftB");

        rightA.setDirection(drivetrainReverse ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        rightB.setDirection(drivetrainReverse ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        leftA.setDirection(drivetrainReverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        leftB.setDirection(drivetrainReverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        rightA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hwmap.get(DcMotor.class, "arm");
        succc = hwmap.get(DcMotor.class, "intake");
        flup = hwmap.get(Servo.class, "flup");

        arm.setDirection(DcMotor.Direction.FORWARD);
        succc.setDirection(DcMotor.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        capLift = hwmap.get(DcMotor.class, "capLift");

        ddr = hwmap.get(CRServo.class, "ddr");*/

        BNO055IMU.Parameters parameters             = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled                   = false;
        parameters.loggingTag                       = "IMU";

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        WebcamName webcamName = hwmap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
    }
}
