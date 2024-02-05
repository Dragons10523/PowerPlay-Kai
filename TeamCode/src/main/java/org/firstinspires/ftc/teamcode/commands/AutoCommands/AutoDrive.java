package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystems;
import org.firstinspires.ftc.teamcode.vision.ColorPipeline;

import java.util.function.BooleanSupplier;

public class AutoDrive extends CommandBase {
    Mushu mushu;

    Boolean isFinished = false;
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
    }

    public void execute(){



        mushu.drivetrainMode(Motor.RunMode.VelocityControl);


        PIDFController pidf = new PIDFController(.2,.1,.05,17);

        double kP = pidf.getP();
        double kI = pidf.getI();
        double kD = pidf.getD();
        double kF = pidf.getF();

        pidf.setPIDF(kP,kI,kD,kF);


//        mushu.frontLeft.setPositionTolerance(5 * TICKS_PER_INCH);
//        mushu.frontRight.setPositionTolerance(5 * TICKS_PER_INCH);
//        mushu.backLeft.setPositionTolerance(5 * TICKS_PER_INCH);
//        mushu.backRight.setPositionTolerance(5 * TICKS_PER_INCH);
//
//        double strafe;
//        double drive;
//
//        strafe = Math.cos(theta) - Math.sin(theta);
//        drive = Math.sin(theta) + Math.cos(theta);
//        double[] speeds = {
//                (drive + strafe),
//                (drive - strafe),
//                (drive - strafe),
//                (drive + strafe)};

        targetDistance = (int) (targetDistance * (TICKS_PER_INCH));

        double output = pidf.calculate( mushu.frontLeft.getCurrentPosition(), targetDistance);

        pidf.setSetPoint(output);

       // setTargetPosition(targetDistance, targetDistance, targetDistance, targetDistance); // might MIGHT be able to multiply by speeds 0-3

        if(targetDistance < 0){
            power *= -1;
        }

        while(!pidf.atSetPoint() && !isStopRequested.getAsBoolean()){
            double outputFLM = pidf.calculate(
                    mushu.frontLeft.getCurrentPosition()
            );
            double outputFRM = pidf.calculate(
                    mushu.frontRight.getCurrentPosition()
            );
            double outputBLM = pidf.calculate(
                    mushu.backLeft.getCurrentPosition()
            );
            double outputBRM = pidf.calculate(
                    mushu.backRight.getCurrentPosition()
            );


            setMotorPower(outputFRM, outputFLM, outputBRM, outputBLM);
            
            telemetry.addData("targetDistance", targetDistance);
            telemetry.addData("FLM", mushu.frontLeft.getCurrentPosition());
            telemetry.addData("FRM", mushu.frontRight.getCurrentPosition());
            telemetry.addData("BLM", mushu.backLeft.getCurrentPosition());
            telemetry.addData("BRM", mushu.backRight.getCurrentPosition());
            telemetry.addData("output", outputFLM);

            telemetry.update();
        }
        mushu.mecanum.stop();

        isFinished = true;

    }
    @Override
    public void end(boolean interrupted){
        mushu.mecanum.stop();

    }
    @Override
    public boolean isFinished(){
        return isStopRequested.getAsBoolean() || isAtTarget();
    }
    public boolean isAtTarget(){
        return mushu.frontLeft.atTargetPosition() || mushu.frontRight.atTargetPosition() || mushu.backRight.atTargetPosition() || mushu.backLeft.atTargetPosition();
    }
    void setMotorPower(double FRM, double FLM, double BRM, double BLM){
        mushu.frontRight.set(FRM);
        mushu.frontLeft.set(FLM);
        mushu.backRight.set(BRM);
        mushu.backLeft.set(BLM);
    }
    double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI *  ticks / 560;

    }


}

