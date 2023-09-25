package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ToolSubsystem extends SubsystemBase{
    private final DcMotor arm;
    private final CRServo intakeServo;
    private final CRServo omniIntake;


    public ToolSubsystem(HardwareMap hMap, String CRServoName,
                         String motorName, String omniName){
        arm = hMap.get(DcMotor.class, motorName);
        intakeServo = hMap.get(CRServo.class, CRServoName);
        omniIntake = hMap.get(CRServo.class, omniName);
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
    public void spinIntake(boolean SPIN){
        if(SPIN){intakeServo.setPower(1);}

        else {intakeServo.setPower(0);}
    }
    public void intake2Extake(boolean SPIN){
        if(SPIN){omniIntake.setPower(1);}

        else {intakeServo.setPower(0);}
    }

}
