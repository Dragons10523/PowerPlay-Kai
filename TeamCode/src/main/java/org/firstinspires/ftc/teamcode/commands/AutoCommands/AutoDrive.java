package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;

import java.util.function.BooleanSupplier;

public class AutoDrive extends CommandBase {
    Mushu mushu;

    static Boolean isFinished = false;
    BooleanSupplier isStopRequested;
    public static final int WHEEL_RADIUS = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 /  WHEEL_CIRCUMFERENCE_INCH;

    int targetDistance = 0;
    double power;
    int theta;
    MecanumDriveSubsystems sub;
    Telemetry telemetry;


    public AutoDrive(double power, int targetDistance, int theta, MecanumDriveSubsystems sub, Mushu mushu, Telemetry telemetry, BooleanSupplier isStopRequested){
        this.targetDistance =  targetDistance;
        this.power = power;
        this.theta = theta;
        this.sub = sub;
        this.mushu = mushu;
        this.telemetry = telemetry;
        this.isStopRequested = isStopRequested;

    }

    public void initialize(){
        mushu.frontRight.stopAndResetEncoder();
        mushu.frontLeft.stopAndResetEncoder();
        mushu.backRight.stopAndResetEncoder();
        mushu.backLeft.stopAndResetEncoder();
        isFinished = false;

    }

    public void execute(){
        mushu.frontRight.stopAndResetEncoder();
        mushu.frontLeft.stopAndResetEncoder();
        mushu.backRight.stopAndResetEncoder();
        mushu.backLeft.stopAndResetEncoder();

        isStopRequested.getAsBoolean();

        mushu.drivetrainMode(Motor.RunMode.PositionControl);

        mushu.frontLeft.setPositionCoefficient(.1);
        mushu.frontRight.setPositionCoefficient(.1);
        mushu.backLeft.setPositionCoefficient(.1);
        mushu.backRight.setPositionCoefficient(.1);

        targetDistance = (int) (targetDistance * (TICKS_PER_INCH));


        setTargetPosition(targetDistance, targetDistance, targetDistance, targetDistance); // might MIGHT be able to multiply by speeds 0-3


        mushu.frontLeft.setPositionTolerance(.5 * TICKS_PER_INCH);
        mushu.frontRight.setPositionTolerance(.5 * TICKS_PER_INCH);
        mushu.backLeft.setPositionTolerance(.5 * TICKS_PER_INCH);
        mushu.backRight.setPositionTolerance(.5 * TICKS_PER_INCH);

        do{


            setMotorPower(power, power, power, power);
            
            telemetry.addData("targetDistance", targetDistance);
            telemetry.addData("FLM", mushu.frontLeft.getCurrentPosition());
            telemetry.addData("FRM", mushu.frontRight.getCurrentPosition());
            telemetry.addData("BLM", mushu.backLeft.getCurrentPosition());
            telemetry.addData("BRM", mushu.backRight.getCurrentPosition());
            telemetry.addData("power", mushu.frontLeft.get());
            //telemetry.addData()

            telemetry.update();
        }
        while(!mushu.frontLeft.atTargetPosition() && !mushu.frontRight.atTargetPosition() && !isStopRequested.getAsBoolean());
        mushu.mecanum.stop();

        this.cancel();

        isFinished = true;
    }
    @Override
    public void end(boolean interrupted){
        mushu.mecanum.stop();
        mushu.frontRight.stopAndResetEncoder();
        mushu.frontLeft.stopAndResetEncoder();
        mushu.backRight.stopAndResetEncoder();
        mushu.backLeft.stopAndResetEncoder();

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }
    @Override
    public boolean isFinished(){
        return isStopRequested.getAsBoolean() || isAtTarget();
    }
    public static boolean isAtTarget(){
        return isFinished;
    }
    void setMotorPower(double FRM, double FLM, double BRM, double BLM){
        double powerReduce = (targetDistance - (double) mushu.frontLeft.getCurrentPosition()) / (8 * TICKS_PER_INCH);

        telemetry.addData("powerReduce", powerReduce);
        telemetry.addData("power", constrainPower((FLM * powerReduce)));

        mushu.frontRight.set(Math.max((constrainPower((FRM * powerReduce))), 0.15));
        mushu.frontLeft.set(Math.max((constrainPower((FLM * powerReduce))), 0.15));
        mushu.backRight.set(Math.max((constrainPower((BRM * powerReduce))), 0.15));
        mushu.backLeft.set(Math.max((constrainPower((BLM * powerReduce))), 0.15));
    }
    double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI *  ticks / 560;

    }
    void setTargetPosition(int FLM, int FRM, int BLM, int BRM){
        mushu.frontLeft.setTargetPosition(FLM);
        mushu.frontRight.setTargetPosition(FRM);
        mushu.backLeft.setTargetPosition(BLM);
        mushu.backRight.setTargetPosition(BRM);
    }
    double constrainPower(double input){
        return Math.max(Math.min(input, .5)  ,.2);
    }


}

