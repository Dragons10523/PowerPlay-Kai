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
            toolSubsystem.manualIntake(Math.max(0, toolGamepad.getRightY()));
            isLimitingIn = true;
        }
        else {
            toolSubsystem.manualIntake(toolGamepad.getRightY());
            isLimitingIn = false;
        }
        if(extakeStartPos >= mushu.extakeArm.getCurrentPosition() + 5){
            toolSubsystem.manualExtake(Math.max(0, toolGamepad.getLeftY()));
            isLimitingEx = true;
        }
        else {
            toolSubsystem.manualExtake(toolGamepad.getLeftY());
            isLimitingEx = false;
        }

        telemetry.addData("intakeArmPos", mushu.intakeArm.getCurrentPosition());
        telemetry.addData("isLimitingIn", isLimitingIn);
        telemetry.addData("extakeArmPos", mushu.extakeArm.getCurrentPosition());
        telemetry.addData("isLimitingEx", isLimitingEx);
        telemetry.update();




    }

    public static class Intake extends CommandBase{
        GamepadEx gamepad;
        double intakePower;
        double extakePower;


        InExtakeSub sub;
        public Intake(GamepadEx gamepad, InExtakeSub sub){
            this.gamepad = gamepad;
            this.sub = sub;

        }
        @Override
        public void execute(){
           intakePower = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER); // INTAKE IS LEFT TRIGGER
            sub.runIN(intakePower);
        }
    }
    public static class extakeSpin extends CommandBase{
        InExtakeSub sub;
        double power;
        public extakeSpin(InExtakeSub sub, double power){this.sub = sub; this.power = power;}
        public void execute(){
            sub.runEX(power);
        }
    }
    public static class flipServo extends CommandBase{
        InExtakeSub sub;
        public flipServo(InExtakeSub sub){this.sub = sub;}
        public void initialize(){sub.flipGate();}
    }
    public static class retractServo extends CommandBase{
        InExtakeSub sub;
        public retractServo(InExtakeSub sub){this.sub = sub;}
        public void initialize() {sub.retractGate();}
    }
}
