package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mushu;

public class ToolSubsystem extends SubsystemBase{
  Mushu mushu;

  GamepadEx toolGamepad;

    public ToolSubsystem(GamepadEx toolGamepad, Mushu mushu){
        this.toolGamepad = toolGamepad;
        this.mushu = mushu;
    }


     // @param targetPosition target position. -1 for default

    public void setArmExtakePosition(int targetPosition){
        if(targetPosition == -1) {
            targetPosition = 1000;
        }
        mushu.extakeArm.setTargetPosition(targetPosition);
        mushu.extakeArm.setRunMode(Motor.RunMode.PositionControl);
    }
    public void setArmIntakePosition(int targetPosition){
        if(targetPosition == -1){
            targetPosition = 100;
        }
        mushu.intakeArm.setTargetPosition(targetPosition);
        mushu.extakeArm.setRunMode(Motor.RunMode.PositionControl);
    }
    //TODO: make sure can't go below 0
    public void manualExtake(double power){
        mushu.extakeArm.setRunMode(Motor.RunMode.RawPower);
        mushu.extakeArm.set(power);
    }
    public void manualIntake(double power){
        mushu.intakeArm.setRunMode(Motor.RunMode.RawPower);
        mushu.extakeArm.set(power);
    }
    public void hang(double power){
        mushu.hangMotor.set(power);
    }

    public void spinOmni() { mushu.omniServo.set(1); }
    public void stopOmni() { mushu.omniServo.set(0); }

    public int getIntakePosition(){ return mushu.intakeArm.getCurrentPosition(); }
    public int getExtakePosition(){ return mushu.extakeArm.getCurrentPosition(); }

    public void resetToolMotors() {
        mushu.intakeArm.stopAndResetEncoder();
        mushu.extakeArm.stopAndResetEncoder();
    }


}
