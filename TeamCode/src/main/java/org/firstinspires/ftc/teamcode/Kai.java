package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.drivetrain.MecanumDrivetrain;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.processors.Deadwheels;

public class Kai {
    // Also used for deadwheels. Use back motors for both y axis, front left for the x axis
    public final MecanumDrivetrain drivetrain;

    public final DcMotorEx liftExtension;
    public final DcMotorEx turntable;
    public final DcMotor armLiftA, armLiftB;
    public final Servo claw, clawTwist;

    public final Rev2mDistanceSensor rightDist, leftDist;
    public final RevTouchSensor extensionLimit;

    public final IMU imu;

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

        liftExtension = hwmap.get(DcMotorEx.class, "horizontalLift");
        turntable = hwmap.get(DcMotorEx.class, "turntable");
        armLiftA = hwmap.get(DcMotor.class, "armLiftA");
        armLiftB = hwmap.get(DcMotor.class, "armLiftB");

        claw = hwmap.get(Servo.class, "claw");
        clawTwist = hwmap.get(Servo.class, "clawTwist");

        rightDist = hwmap.get(Rev2mDistanceSensor.class, "rightDist");
        leftDist = hwmap.get(Rev2mDistanceSensor.class, "leftDist");

        extensionLimit = hwmap.get(RevTouchSensor.class, "extensionLimit");

        imu = hwmap.get(IMU.class, "imu");

        WebcamName frontWebcamName = hwmap.get(WebcamName.class, "Webcam 1");
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(frontWebcamName);

        // Set motor directions
        armLiftA.setDirection(DcMotorSimple.Direction.REVERSE);
        armLiftB.setDirection(DcMotorSimple.Direction.REVERSE);
        turntable.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor behavior
        drivetrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armLiftA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up encoders
        liftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try {
            Thread.sleep(50);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }

        liftExtension.setTargetPosition(0);
        turntable.setTargetPosition(0);
        armLiftA.setTargetPosition(0);
        armLiftB.setTargetPosition(0);

        turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftExtension.setPower(1);
        turntable.setPower(0);
        armLiftA.setPower(1);
        armLiftB.setPower(1);

        liftExtension.setTargetPositionTolerance(69);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        deadwheels = new Deadwheels(drivetrain.driveMotors[2], drivetrain.driveMotors[1], drivetrain.driveMotors[0], 2.82, 1.46314322832412201, Math.PI/2048);
    }

    public double getHeading(){
        // Maf angles, not game
        return deadwheels.currentAngle;
    }
}
