package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.drivetrain.MecanumDrivetrain;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.processors.Deadwheels;

public class Kai {
    // Also used for deadwheels. Use back motors for both y axis, front left for the x axis
    public final MecanumDrivetrain drivetrain;

    public final DcMotor liftExtension;
    public final DcMotorEx turntable;
    public final DcMotor armLiftA, armLiftB;
    public final Servo claw, clawTwist;

    public final DistanceSensor frontDist, rightDist, leftDist, backDist;
    public final DistanceSensor clawSensor;

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

        liftExtension = hwmap.get(DcMotor.class, "horizontalLift");
        turntable = hwmap.get(DcMotorEx.class, "turntable");
        armLiftA = hwmap.get(DcMotor.class, "armLiftA");
        armLiftB = hwmap.get(DcMotor.class, "armLiftB");

        claw = hwmap.get(Servo.class, "claw");
        clawTwist = hwmap.get(Servo.class, "clawTwist");

        frontDist = hwmap.get(DistanceSensor.class, "frontDist");
        rightDist = hwmap.get(DistanceSensor.class, "rightDist");
        leftDist = hwmap.get(DistanceSensor.class, "leftDist");
        backDist = hwmap.get(DistanceSensor.class, "backDist");

        clawSensor = hwmap.get(DistanceSensor.class, "clawSensor");

        WebcamName frontWebcamName = hwmap.get(WebcamName.class, "Webcam 1");
        //frontCamera = OpenCvCameraFactory.getInstance().createWebcam(frontWebcamName);
        frontCamera = null;

        // Set motor directions
        armLiftA.setDirection(DcMotorSimple.Direction.REVERSE);
        armLiftB.setDirection(DcMotorSimple.Direction.REVERSE);
        turntable.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor behavior
        drivetrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up encoders
        liftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftExtension.setTargetPosition(0);
        turntable.setTargetPosition(0);
        armLiftA.setTargetPosition(0);
        armLiftB.setTargetPosition(0);

        //turntable.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(2, 0, 0, 0));

        liftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftExtension.setPower(1);
        turntable.setPower(0.35);
        armLiftA.setPower(1);
        armLiftB.setPower(1);

        deadwheels = new Deadwheels(drivetrain.driveMotors[2], drivetrain.driveMotors[1], drivetrain.driveMotors[0], 2.82, .43, Math.PI/2048);
    }

    public double getHeading(){
        // Maf angles, not game
        return deadwheels.currentAngle;
    }
}
