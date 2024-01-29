package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.vision.ColorPipeline;

public class AutoDrive extends CommandBase {
    Mushu mushu;

    Boolean isFinished = false;
    public static final int WHEEL_RADIUS_INCH = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS_INCH;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 /  WHEEL_CIRCUMFERENCE_INCH;

    int targetDistance;
    double power;
    int theta;
    MecanumDriveSubsystems sub;


    public AutoDrive(double power, int targetDistance, int theta, MecanumDriveSubsystems sub, Mushu mushu){
        this.targetDistance =  targetDistance;
        this.power = power;
        this.theta = theta;
        this.sub = sub;
        this.mushu = mushu;

    }
    public void execute(){

        mushu.drivetrainMode(Motor.RunMode.PositionControl);

        mushu.frontLeft.setPositionCoefficient(.05); // ALL KP VALUES ARE WRONG AND FROM FTC LIB
        mushu.frontRight.setPositionCoefficient(.05);
        mushu.backLeft.setPositionCoefficient(.05);
        mushu.backRight.setPositionCoefficient(.05);

        double strafe;
        double drive;

        strafe = Math.cos(theta) - Math.sin(theta);
        drive = Math.sin(theta) + Math.cos(theta);
//        double[] speeds = {
//                (drive + strafe),
//                (drive - strafe),
//                (drive - strafe),
//                (drive + strafe)};

        targetDistance = (int) (targetDistance * TICKS_PER_INCH);

        setTargetPosition(targetDistance, targetDistance, targetDistance, targetDistance); // might MIGHT be able to multiply by speeds 0-3

        while(!isAtTarget()){
            sub.manualDrive(-power, -power, -power, -power);
        }
        mushu.mecanum.stop();

        isFinished = true;

    }
    @Override
    public void end(boolean interrupted){
        mushu.mecanum.stop();
        mushu.stopMotors();
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }
    public boolean isAtTarget(){
        return mushu.frontLeft.atTargetPosition() && mushu.frontRight.atTargetPosition()
                && mushu.backLeft.atTargetPosition() && mushu.backRight.atTargetPosition();
    }
    public void setTargetPosition(int FRM, int FLM, int BRM, int BLM){
        mushu.frontRight.setTargetPosition(FRM);
        mushu.frontLeft.setTargetPosition(FLM);
        mushu.backRight.setTargetPosition(BRM);
        mushu.backLeft.setTargetPosition(BLM);
    }


}

