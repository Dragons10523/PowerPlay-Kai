package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class Mushu extends Robot {
    private static Mushu instance;

    public GamepadEx driverGamepad;
    public GamepadEx toolGamepad;

    public DifferentialDrive drivetrain;
    public MecanumDrive mecanum;

    public static Mushu GetInstance(CommandOpMode opMode) {
        if(instance == null)
        {
            instance = new Mushu(opMode);
        }

        return instance;
    }

    private Mushu(CommandOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        Motor frontLeft = new Motor(hardwareMap, "FrontLeft");
        Motor frontRight = new Motor(hardwareMap, "FrontRight");
        Motor backLeft = new Motor(hardwareMap, "BackLeft");
        Motor backRight = new Motor(hardwareMap, "BackRight");
        Motor extake = new Motor(hardwareMap, "extake");
        Motor arm = new Motor(hardwareMap, "arm");
        CRServo intakeServo = new CRServo(hardwareMap, "intakeServo");
        CRServo omniServo = new CRServo(hardwareMap, "omniServo");



        mecanum  = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        driverGamepad = new GamepadEx(opMode.gamepad1);
        toolGamepad = new GamepadEx(opMode.gamepad2);
    }
}
