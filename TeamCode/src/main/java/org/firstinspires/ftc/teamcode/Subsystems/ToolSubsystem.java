package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ToolSubsystem extends SubsystemBase{
    private final DcMotor arm;
    private final Servo intakeServo;
    private final Servo omniIntake;


    public ToolSubsystem(HardwareMap hMap, String servoName,
                         String motorName, String omniName){
        arm = hMap.get(DcMotor.class, motorName);
        intakeServo = hMap.get(Servo.class, servoName);
        omniIntake = hMap.get(Servo.class, omniName);
    }
    public void extendArm(int targetPosition, boolean defaultExtension){ // defaultExtension extends to max position
        arm.setTargetPosition(targetPosition);
        if(defaultExtension){
            arm.setTargetPosition(1000);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void retractArm(){
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}
