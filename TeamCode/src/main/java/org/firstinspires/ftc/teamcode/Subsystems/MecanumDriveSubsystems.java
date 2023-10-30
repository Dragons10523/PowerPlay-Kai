package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumDriveSubsystems extends SubsystemBase{
    private final MecanumDrive mecanum;
    private final Motor frontLeft;
    private final Motor frontRight;
    private final Motor bottomLeft;
    private final Motor bottomRight;


    public MecanumDriveSubsystems(final HardwareMap hMap, String frontLeft,
                                  String frontRight, String bottomLeft,
                                  String bottomRight, MecanumDrive mecanum){

        this.mecanum = mecanum;
        this.frontLeft = hMap.get(Motor.class, frontLeft);
        this.frontRight = hMap.get(Motor.class, frontRight);
        this.bottomLeft = hMap.get(Motor.class, bottomLeft);
        this.bottomRight = hMap.get(Motor.class, bottomRight);

    }

    public void mecanumDrive(){
        mecanum.driveFieldCentric();
    }


}
