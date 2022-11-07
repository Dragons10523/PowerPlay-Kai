package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.drivetrain.MecanumDrivetrain;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class Kai {
    // Also used for deadwheels. Use back motors for both y axis, front left for the x axis
    public final MecanumDrivetrain drivetrain;

    public final DcMotor horizontalLift;
    public final DcMotorEx turntable;
    public final DcMotor armLiftA, armLiftB;
    public final Servo claw, clawFlup, clawTwist;

    public final DistanceSensor frontDist, rightDist, leftDist, backDist;
    public final DistanceSensor clawSensor;

    public final BNO055IMU imu;

    public OpenCvWebcam frontCamera;

    public Deadwheels deadwheels;
    public HardwareMap hwmap;

    public Kai(HardwareMap hwmap) {
        this.hwmap = hwmap;

        // Map electronics
        drivetrain = new MecanumDrivetrain(
                hwmap.get(DcMotor.class, "frontLeft"), hwmap.get(DcMotor.class, "frontRight"),
                hwmap.get(DcMotor.class, "backLeft"), hwmap.get(DcMotor.class, "backRight")
        );

        horizontalLift = hwmap.get(DcMotor.class, "horizontalLift");
        turntable = hwmap.get(DcMotorEx.class, "turntable");
        armLiftA = hwmap.get(DcMotor.class, "armLiftA");
        armLiftB = hwmap.get(DcMotor.class, "armLiftB");

        claw = hwmap.get(Servo.class, "claw");
        clawFlup = hwmap.get(Servo.class, "clawFlup");
        clawTwist = hwmap.get(Servo.class, "clawTwist");

        frontDist = hwmap.get(DistanceSensor.class, "frontDist");
        rightDist = hwmap.get(DistanceSensor.class, "rightDist");
        leftDist = hwmap.get(DistanceSensor.class, "leftDist");
        backDist = hwmap.get(DistanceSensor.class, "backDist");

        clawSensor = hwmap.get(DistanceSensor.class, "clawSensor");

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

        // Set motor behavior
        drivetrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up encoders
        horizontalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horizontalLift.setTargetPosition(0);
        turntable.setTargetPosition(0);
        armLiftA.setTargetPosition(0);
        armLiftB.setTargetPosition(0);

        horizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        horizontalLift.setPower(1);
        turntable.setPower(1);
        armLiftA.setPower(1);
        armLiftB.setPower(1);

        deadwheels = new Deadwheels(drivetrain.driveMotors[2], drivetrain.driveMotors[3], drivetrain.driveMotors[0], 7, 0, Math.PI/4096);
    }

    public double getHeading(){
        // Maf angles, not game
        return deadwheels.currentAngle;
    }
}
