package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Utils {
    RobotClass robot;
    public Utils(RobotClass robot){
        this.robot = robot;
    }
    /*
       define global enums here
        v

        ex.
        public enum Pos{
           OPEN,
           CLOSED
        }

        ^
    */
    public enum LiftMode{
        RAW_POWER,
        ENCODER_DRIVE
    }
    public enum ArmFlipState {
        UP,
        GROUND,
        MIDDLE,
    }
    public enum ArmState {
        IN,
        EXTENDED,
    }
    public enum LiftState {
        GROUND,
        LOW,
        HIGH,
    }
    public enum DriveMode {
        GLOBAL,
        LOCAL
    }
    public enum FieldSide {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static DriveMode driveMode = DriveMode.GLOBAL;
    public static LiftMode liftMode = LiftMode.RAW_POWER;
    public static LiftState liftState = LiftState.GROUND;
    public static ArmState armState = ArmState.IN;
    public static ArmFlipState armFlipState = ArmFlipState.UP;
    public static boolean slowMode = false;
    ElapsedTime elapsedTime = new ElapsedTime();
    double timeWhenPressed = 0;

    public void setMotorMode(RobotClass.MOTORS motor, DcMotorEx.RunMode runMode){
        robot.Motors.get(motor).setMode(runMode);
    }
    public void setMotorPower(RobotClass.MOTORS motor, double power){
        robot.Motors.get(motor).setPower(power);
    }
    public void runToLiftPos(){

        switch(liftState){
            case GROUND:
                robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setTargetPosition(0);
                robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setTargetPosition(0);
                break;
            case LOW:
                robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setTargetPosition(4034);
                robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setTargetPosition(5164);
                break;
            case HIGH:
                robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setTargetPosition(7621);
                robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setTargetPosition(10825);
                break;
        }
        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setPower(1);
        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setPower(1);
    }
    public void mecanumDrive(double leftY, double leftX, double turn) {
        if (Utils.driveMode == Utils.DriveMode.GLOBAL) {
            robot.drivetrain.mecanumDriveGlobal(leftY, leftX, turn, robot.getHeading());
        }
        else {
            robot.drivetrain.mecanumDriveLocal(leftY, leftX, turn);
        }


    }
    boolean firstPressLiftPower = true;
    public void liftPower(double liftPower){
        if(liftPower == 0){
            firstPressLiftPower = true;
        }
        if(Utils.liftMode == Utils.LiftMode.ENCODER_DRIVE){
            if(liftPower == 1 && firstPressLiftPower){
                firstPressLiftPower = false;
                switch(Utils.liftState) {
                    case GROUND:
                        Utils.liftState = Utils.LiftState.LOW;
                        break;
                    case LOW:
                        Utils.liftState = Utils.LiftState.HIGH;
                        break;
                }
                runToLiftPos();
            }
            if(liftPower == -1 && firstPressLiftPower){
                firstPressLiftPower = false;
                switch(Utils.liftState){
                    case HIGH:
                        Utils.liftState = Utils.LiftState.LOW;
                        break;
                    case LOW:
                        Utils.liftState = Utils.LiftState.GROUND;
                        break;
                }
                runToLiftPos();
            }
        }
        else{
            robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setPower(liftPower);
            robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setPower(liftPower);
        }
    }
    double[] arm_leftPos = {0.67, 0.66, 0.65, 0.64, 0.63, 0.62, 0.61, 0.60, 0.59,
                            0.58, 0.57, 0.56, 0.55, 0.54, 0.53, 0.52, 0.51, 0.50,
                            0.49, 0.48, 0.47, 0.46, 0.45, 0.43, 0.42};// in to out
    double[] arm_rightPos = {0.52, 0.52, 0.53, 0.54, 0.55, 0.56, 0.57, 0.58, 0.59,
                             0.60, 0.61, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68,
                             0.69, 0.70, 0.71, 0.72, 0.73, 0.74, 0.76};// in to out
    int index = 0;
    double timeAtLastCall = 0;
    public void extendAndRetractArm(double armPower){
        if(armPower > 0 && timeAtLastCall + Math.abs(10 / armPower) < elapsedTime.milliseconds() ){
            timeAtLastCall = elapsedTime.milliseconds();
            if(index != 24){
                index++;
            }
        }
        if(armPower < 0 && timeAtLastCall + Math.abs(10 / armPower) < elapsedTime.milliseconds() ){
            timeAtLastCall = elapsedTime.milliseconds();
            if(index != 0){
                index--;
            }
        }
        if(armPower != 0){
            robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(arm_leftPos[index]);
            robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(arm_rightPos[index]);
        }
    }
    public void flipArm(double power, boolean button){
        if(power != 0){
            robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(power/2);
        if(button){
            robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setTargetPosition(0);
            robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            double kP = (double) robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition() / 100;
            kP = Math.max(.1 ,kP);
            robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(kP);
        }
    }
    public void powerIntake(double power){
        power = Math.min(.75, Math.max(-.75, power));
        robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(power);
    }
    boolean firstPressResetIMU = true;
    public void resetIMU(boolean button) {
        if(button) robot.resetIMU();
    }
    boolean firstPressSwitchDriveMode = true;
    public void switchDriveMode(boolean button) {
        if(!button){
            firstPressSwitchDriveMode = true;
        }
        if (button && firstPressSwitchDriveMode) {
            firstPressSwitchDriveMode = false;
            switch (Utils.driveMode) {
                case GLOBAL:
                    Utils.driveMode = Utils.DriveMode.LOCAL;
                    break;
                case LOCAL:
                    Utils.driveMode = Utils.DriveMode.GLOBAL;
                    break;
            }
        }
    }
    boolean firstPressFlipBucket = true;
    public void flipBucket(boolean button){
        if(!button){
            firstPressFlipBucket = true;
        }
        if(button && firstPressFlipBucket) {
            firstPressFlipBucket = false;
            timeWhenPressed = elapsedTime.seconds();
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0);
        }
        if(elapsedTime.seconds() - 3 > timeWhenPressed ){
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(.5);
        }
    }
    boolean firstPressSwitchLiftMode = true;
    public void switchLiftMode(boolean button){
        if(!button){
            firstPressSwitchLiftMode = true;
        }
        if(button && firstPressSwitchLiftMode){
            firstPressSwitchLiftMode = false;
            switch (Utils.liftMode) {
                case ENCODER_DRIVE:
                    Utils.liftMode = Utils.LiftMode.RAW_POWER;
                    robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    break;
                case RAW_POWER:
                    Utils.liftMode = Utils.LiftMode.ENCODER_DRIVE;
                    robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    break;
            }
        }
    }
    public void deathWiggle(boolean button){
        if(button){
            if(elapsedTime.milliseconds() % 150 < 75){
                robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(.8);
                robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(.8);
                robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(-.8);
                robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(-.8);
            }
            if(elapsedTime.milliseconds() % 150 > 75){
                robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(-.8);
                robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(-.8);
                robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(.8);
                robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(.8);
            }
        }
    }
    boolean firstPressSwitchSlowMode = true;
    public void switchSlowMode(boolean button){
        if(!button){
            firstPressSwitchSlowMode = true;
        }
        if(button && firstPressSwitchSlowMode){
            firstPressSwitchSlowMode = false;
            slowMode = !slowMode;
        }
    }
}
