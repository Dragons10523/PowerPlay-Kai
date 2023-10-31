package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mushu;

public class ToolSubsystem extends SubsystemBase{
  Mushu mushu;

    public ToolSubsystem(){

    }


     // @param targetPosition target position. -1 for default

    public void setArmPosition(int targetPosition){
        if(targetPosition < 0) { targetPosition = 1000; }
        mushu.arm.setTargetPosition(targetPosition);
        mushu.arm.setRunMode(Motor.RunMode.PositionControl);
    }


    public void spinIntake() { mushu.intakeServo.set(1); }
    public void stopIntake() { mushu.intakeServo.set(0); }

    public void spinOmni() { mushu.omniServo.set(1); }
    public void stopOmni() { mushu.omniServo.set(0); }


}
