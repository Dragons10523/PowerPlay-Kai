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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class Mushu extends Robot {
    private static Mushu instance;

    public GamepadEx driverGamepad;
    public GamepadEx toolGamepad;
    public IMU imu;

    public DifferentialDrive drivetrain;
    public MecanumDrive mecanum;
    public Motor frontLeft;
    public Motor frontRight;
    public Motor backLeft;
    public Motor backRight;
    public Motor extake;
    public Motor arm;
    public CRServo intakeServo;
    public CRServo omniServo;

    public static Mushu GetInstance(CommandOpMode opMode) {
        if(instance == null)
        {
            instance = new Mushu(opMode);
        }

        return instance;
    }

    public Mushu(CommandOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        frontLeft = new Motor(hardwareMap, "FrontLeft");
        frontRight = new Motor(hardwareMap, "FrontRight");
        backLeft = new Motor(hardwareMap, "BackLeft");
        backRight = new Motor(hardwareMap, "BackRight");
        extake = new Motor(hardwareMap, "extake");
        arm = new Motor(hardwareMap, "arm");
        intakeServo = new CRServo(hardwareMap, "intakeServo");
        omniServo = new CRServo(hardwareMap, "omniServo");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));




        mecanum  = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        driverGamepad = new GamepadEx(opMode.gamepad1);
        toolGamepad = new GamepadEx(opMode.gamepad2);
    }
    public double getHeading(){
        imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double tempValue =
        return
    }
}
