package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class Mushu extends Robot {
    private static Mushu instance;

    public GamepadEx driverGamepad;
    public GamepadEx toolGamepad;
    //public BHI260IMU imu;
    public IMU imu;

    public MecanumDrive mecanum;
    public Motor frontLeft, frontRight, backLeft, backRight;
    public Motor extakeArm, intakeArm;
    public CRServo intakeServo, omniServo;
    double yaw;
    double pitch;
    double roll;

    public static Mushu GetInstance(CommandOpMode opMode) {
        if(instance == null)
        {
            instance = new Mushu(opMode);
        }

        return instance;
    }

    public Mushu(CommandOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        // TODO: set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        frontLeft = new Motor(hardwareMap, "FrontLeft");
        frontRight = new Motor(hardwareMap, "FrontRight");
        backLeft = new Motor(hardwareMap, "BackLeft");
        backRight = new Motor(hardwareMap, "BackRight");
        extakeArm = new Motor(hardwareMap, "extake");
        intakeArm = new Motor(hardwareMap, "arm");
        intakeServo = new CRServo(hardwareMap, "intakeServo");
        omniServo = new CRServo(hardwareMap, "omniServo");
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
        Orientation Theta = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
       return Theta.firstAngle;
    }
   // public void resetIMU(){
   //    imu.resetYaw();
  //  }


}
