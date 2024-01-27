package org.firstinspires.ftc.teamcode.commands;



import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.InExtakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.ToolSubsystem;

public class Tool extends CommandBase {
    private final ToolSubsystem toolSubsystem;
    GamepadEx toolGamepad;

    Mushu mushu;

    int intakeStartPos;
    int extakeStartPos;
    Telemetry telemetry;
    boolean isLimitingIn = false;
    boolean isLimitingEx = false;
    public Tool(GamepadEx toolGamepad, ToolSubsystem subsystem, Mushu mushu, Telemetry telemetry){
        toolSubsystem = subsystem;
        this.toolGamepad = toolGamepad;
        this.mushu = mushu;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(){
       // mushu.intakeArm.stopAndResetEncoder();
       // mushu.extakeServo.stopAndResetEncoder();

        intakeStartPos = mushu.intakeArm.getCurrentPosition();
        extakeStartPos = mushu.extakeArm.getCurrentPosition();

    }
    @Override
    public void execute(){
        if(intakeStartPos >= mushu.intakeArm.getCurrentPosition() + 5){
            toolSubsystem.manualIntake(Math.max(0, toolGamepad.getLeftY()));
            isLimitingIn = true;
        }
        else {
            toolSubsystem.manualIntake(toolGamepad.getLeftY());
            isLimitingIn = false;
        }
        if(extakeStartPos >= mushu.extakeArm.getCurrentPosition() + 5){
            toolSubsystem.manualExtake(Math.max(0, toolGamepad.getRightY()));
            isLimitingEx = true;
        }
        else {
            toolSubsystem.manualExtake(toolGamepad.getRightY());
            isLimitingEx = false;
        }
    }
}
