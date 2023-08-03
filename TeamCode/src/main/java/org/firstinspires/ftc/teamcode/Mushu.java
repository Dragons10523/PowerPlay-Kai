package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mushu extends Robot {
    private static Mushu instance;

    public GamepadEx driverGamepad;
    public GamepadEx toolGamepad;

    public DifferentialDrive drivetrain;

    public static Mushu GetInstance(CommandOpMode opMode) {
        if(instance == null) {
            instance = new Mushu(opMode);
        }

        return instance;
    }

    private Mushu(CommandOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        Motor frontLeft = new Motor(hardwareMap, "FrontLeft");
        Motor frontRight = new Motor(hardwareMap, "FrontRight");
        Motor backLeft = new Motor(hardwareMap, "BackLeft");
        Motor backRight = new Motor(hardwareMap, "BackRight");

        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);

        drivetrain = new DifferentialDrive(leftMotors, rightMotors);

        driverGamepad = new GamepadEx(opMode.gamepad1);
        toolGamepad = new GamepadEx(opMode.gamepad2);
    }
}
