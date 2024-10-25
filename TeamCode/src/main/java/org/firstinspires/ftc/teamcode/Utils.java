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
    public static LiftMode liftMode = LiftMode.ENCODER_DRIVE;
    public static LiftState liftState = LiftState.GROUND;
    public static ArmState armState = ArmState.IN;
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
            case LOW:
                robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setTargetPosition(100);
                robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setTargetPosition(100);
            case HIGH:
                robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setTargetPosition(200);
                robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setTargetPosition(200);
        }

        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void mecanumDrive(double leftX, double leftY, double turn) {
        double heading;
        if (Utils.driveMode == Utils.DriveMode.GLOBAL) {
            heading = robot.opticalSensor.getPosition().h;
        } else {
            heading = 0;
        }

        robot.drivetrain.mecanumDrive(leftY, leftX, turn, heading);
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
            robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setPower(liftPower);
            robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setPower(liftPower);
        }
    }
    boolean firstPressExtendAndRetractArm = true;
    public void extendAndRetractArm(boolean button){
        if(!button){
            firstPressExtendAndRetractArm = true;
        }
        if(button && firstPressExtendAndRetractArm){
            firstPressExtendAndRetractArm = false;
            switch(Utils.armState){
                case IN:
                    Utils.armState = Utils.ArmState.EXTENDED;
                    robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.51);
                    robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.76);
                    break;
                case EXTENDED:
                    Utils.armState = Utils.ArmState.IN;
                    robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.79);
                    robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.51);
                    break;
            }

        }
    }
    public void flipArm(double power){
        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMotorEnable();

        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(power);
    }
    public void powerIntake(double power){
        power = Math.min(.75, Math.max(-.75, power));
        robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(power);
    }
    boolean firstPressResetIMU = true;
    public void resetIMU(boolean button) {
        if(!button){
            firstPressResetIMU = true;
        }
        if(button && firstPressResetIMU){
            firstPressResetIMU = false;
            robot.resetIMU();
        }

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
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(1);
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
    public double getCurrentDrawDriveTrain(CurrentUnit currentUnit){
        if(currentUnit == CurrentUnit.MILLIAMPS){
            return robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).getCurrent(CurrentUnit.MILLIAMPS)
                    + robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).getCurrent(CurrentUnit.MILLIAMPS)
                    + robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).getCurrent(CurrentUnit.MILLIAMPS)
                    + robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).getCurrent(CurrentUnit.MILLIAMPS);
        }
        else{
            return robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).getCurrent(CurrentUnit.AMPS)
                    + robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).getCurrent(CurrentUnit.AMPS)
                    + robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).getCurrent(CurrentUnit.AMPS)
                    + robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).getCurrent(CurrentUnit.AMPS);
        }
    }
}
