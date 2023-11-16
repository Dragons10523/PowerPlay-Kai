package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Mushu;


public class MecanumDriveSubsystems extends SubsystemBase{
    Mushu mushu;
    private final MecanumDrive mecanum;
    private final GamepadEx gamepadEx;
    double rightBumper;
    double leftBumper;

    public MecanumDriveSubsystems(MecanumDrive mecanum, GamepadEx gamepadEx){
        this.mecanum = mecanum;
        this.gamepadEx = gamepadEx;

    }

    public void mecanumDrive(){
       mecanum.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(),
               getBumper(), mushu.getHeading(), false);
    }

    public double getBumper(){
        rightBumper = gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        leftBumper = gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        if (rightBumper > 0 && leftBumper == 0) {
            return rightBumper;
        }
        else if (leftBumper > 0 && rightBumper == 0) {
            return -leftBumper;
        }
        else {
            return 0;
        }
    }

}
