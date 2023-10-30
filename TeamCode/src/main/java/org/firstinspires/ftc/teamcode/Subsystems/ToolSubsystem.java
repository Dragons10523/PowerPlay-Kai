package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ToolSubsystem extends SubsystemBase{
    private final Motor arm;
    private final CRServo intakeServo;
    private final CRServo omniIntake;


    public ToolSubsystem(final HardwareMap hMap, String CRServoName,
                         String motorName, String omniName){
        arm = hMap.get(Motor.class, motorName);
        intakeServo = hMap.get(CRServo.class, CRServoName);
        omniIntake = hMap.get(CRServo.class, omniName);
    }


     // @param targetPosition target position. -1 for default

    public void setArmPosition(int targetPosition){
        if(targetPosition < 0) { targetPosition = 1000; }
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void spinIntake() { intakeServo.setPower(1); }
    public void stopIntake() { intakeServo.setPower(0); }

    public void spinOmni() { omniIntake.setPower(1); }
    public void stopOmni() { omniIntake.setPower(0); }


}
