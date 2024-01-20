package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class Mushu extends Robot {
    private static Mushu instance;

    public GamepadEx driverGamepad;
    public GamepadEx toolGamepad;
    //public BHI260IMU imu;
    public IMU imu;
    public MecanumDrive mecanum;
    public Motor frontLeft, frontRight, backLeft, backRight;
    public Motor extakeArm, intakeArm;
    public Motor hangMotor, intakeMotor;
    public CRServo omniServo, extakeServo;
    public ServoEx gateServo, droneServo;
    public WebcamName webcamName;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public byte[] byteData;
    Orientation Theta;
    public static Mushu GetInstance(CommandOpMode opMode) {
        if(instance == null)
        {
            instance = new Mushu(opMode);
        }

        return instance;
    }

    public Mushu(CommandOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;



        byteData = AprilTagPipeline.APRIL_TAG_INIT_DATA;




        frontLeft = new Motor(hardwareMap, "FrontLeft");
        frontRight = new Motor(hardwareMap, "FrontRight");
        backLeft = new Motor(hardwareMap, "BackLeft");
        backRight = new Motor(hardwareMap, "BackRight");
        extakeArm = new Motor(hardwareMap, "extake");
        intakeArm = new Motor(hardwareMap, "arm");
        hangMotor = new Motor (hardwareMap, "hangMotor");
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        extakeServo = new CRServo(hardwareMap, "extakeServo");
        omniServo = new CRServo(hardwareMap, "omniServo");
        gateServo = new SimpleServo(hardwareMap,"gateServo", 0, 360, AngleUnit.DEGREES);
        droneServo = new SimpleServo(hardwareMap, "droneServo", 0, 180, AngleUnit.DEGREES);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters param;
        param = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        // Initialize IMU directly
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

// Initialize IMU using Parameters
        imu.initialize(param);
       // resetIMU();
    

// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:


        mecanum  = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        driverGamepad = new GamepadEx(opMode.gamepad1);
        toolGamepad = new GamepadEx(opMode.gamepad2);
    }
    public double getHeading(){
        Theta = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return Theta.thirdAngle;
       // first angle PITCH facing towards the front set by gravity
       // second angle ROLL facing towards the front set by gravity
       // third angle YAW facing towards the front
        //https://external-content.duckduckgo.com/iu/?u=http%3A%2F%2Fpepijndevos.nl%2Fimages%2F638px-Yaw_Axis_Corrected.svg.png&f=1&nofb=1&ipt=de4a93d65083eb716c740d6eae10504fbe1af10cdde294d327fca79cd27a8953&ipo=images
    }
   public void resetIMU(){

        imu.resetYaw();
    }
   public void stopMotors() {
       frontLeft.stopMotor();
       frontRight.stopMotor();
       backLeft.stopMotor();
       backRight.stopMotor();
       extakeArm.stopMotor();
       intakeArm.stopMotor();
       hangMotor.stopMotor();
       intakeMotor.stopMotor();
       extakeServo.stop();
       omniServo.stop();
   }


}
