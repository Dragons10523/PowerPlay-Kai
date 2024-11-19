package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Utils {
    RobotClass robot;
    ElapsedTime time = new ElapsedTime();

    public Utils(RobotClass robot) {
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
    public static LiftMode liftMode = LiftMode.ENCODER_DRIVE;
    public static LiftState liftState = LiftState.GROUND;
    public static ArmState armState = ArmState.IN;
    public static ArmFlipState armFlipState = ArmFlipState.UP;
    public static boolean slowMode = false;
    ElapsedTime elapsedTime = new ElapsedTime();
    double timeWhenPressed = 0;
    public void runToLiftPos() {
        robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setTargetPositionTolerance(10);
        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setTargetPositionTolerance(10);
        int distanceToGoal = Math.abs(robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).getTargetPosition()
                -robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).getCurrentPosition());
        switch (liftState) {
            case HIGH:
                if(distanceToGoal > 200){

                }
                else {
                    setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setLiftPower(0);
                }
            case GROUND:
                if(distanceToGoal > 200){
                    setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setLiftPower(.8);
                }
                else {
                    setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setLiftPower(0);
                }
        }
    }
    void setLiftMode(DcMotor.RunMode runMode) {
        robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMode(runMode);
        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMode(runMode);
    }

    void setLiftTargetPos(int targetPos) {
        robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setTargetPosition(targetPos);
        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setTargetPosition(targetPos);
    }

    void setLiftPower(double power) {
        robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setPower(power);
        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setPower(power);
    }
    public void resetLift(boolean button){
        if(button){
            robot.initResetLift();
        }
    }
    public void mecanumDrive(double leftY, double leftX, double turn) {
        if (Utils.driveMode == Utils.DriveMode.GLOBAL) {
            robot.drivetrain.mecanumDriveGlobal(leftY, leftX, turn, robot.getHeading());
        } else {
            robot.drivetrain.mecanumDriveLocal(leftY, leftX, turn);
        }
    }
    double startTime = 0;
    public void liftPower(double liftPower, boolean button) {
        if(button){
            startTime = time.seconds();
            setLiftPower(-1);
        }
        else if (startTime + 2 < time.seconds()){
            setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setLiftPower(liftPower);
        }
    }

    double[] arm_leftPos = {0.69, 0.68, 0.67, 0.66, 0.65, 0.64 ,0.63, 0.62, 0.61,
            0.60, 0.59, 0.58, 0.57, 0.56, 0.55, 0.54, 0.53, 0.52,
            0.51, 0.50, 0.49, 0.48, 0.47, 0.46, 0.45, 0.44};// in to out
    double[] arm_rightPos = {0.52, 0.52, 0.53, 0.54, 0.55, 0.56, 0.57, 0.58, 0.59,
            0.60, 0.61, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68,
            0.69, 0.70, 0.71, 0.72, 0.73, 0.74, 0.76};// in to out
    int index = 0;
    double timeAtLastCall = 0;

    public void extendAndRetractArm(double armPower) {
        if (armPower > 0 && timeAtLastCall + Math.abs(10 / armPower) < elapsedTime.milliseconds()) {
            timeAtLastCall = elapsedTime.milliseconds();
            if (index != 24) {
                index++;
            }
        }
        if (armPower < 0 && timeAtLastCall + Math.abs(10 / armPower) < elapsedTime.milliseconds()) {
            timeAtLastCall = elapsedTime.milliseconds();
            if (index != 0) {
                index--;
            }
        }
        if (armPower != 0) {
            robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(arm_leftPos[index]);
            robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(arm_rightPos[index]);
        }
    }

    boolean firstPressOverRide = true;
    boolean overRideArmLimiter = false;

    public void flipArm(double power, boolean button) {
        if (!button) {
            firstPressOverRide = true;
        }
        if (button && firstPressOverRide) {
            firstPressOverRide = false;
            if (overRideArmLimiter) {
                robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            overRideArmLimiter = !overRideArmLimiter;
        }
        if (!overRideArmLimiter) {
            robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int currentArmPos = robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition();
            double armPower = power / 2;
            if (currentArmPos > 1200) {
                armPower = Math.min(0, armPower);
            }
            if (currentArmPos < 0) {
                armPower = Math.max(0, armPower);
            }
            robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(armPower);
        } else {
            robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(power / 2);
        }
    }

    public void powerIntake(double power) {
        power = Math.min(.75, Math.max(-.75, power));
        power *= .7;
        robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(power);
    }

    public void resetIMU(boolean button) {
        if (button) robot.resetIMU();
    }

    boolean firstPressSwitchDriveMode = true;

    public void switchDriveMode(boolean button) {
        if (!button) {
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

    public void flipBucket(boolean button) {
        if (!button) {
            firstPressFlipBucket = true;
        }
        if (button && firstPressFlipBucket) {
            firstPressFlipBucket = false;
            timeWhenPressed = elapsedTime.seconds();
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0);
        }
        if (elapsedTime.seconds() - 3 > timeWhenPressed) {
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
            if (elapsedTime.milliseconds() % 150 > 75) {
                robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(-.8);
                robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(-.8);
                robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(.8);
                robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(.8);
            }
        }
    }

    boolean firstPressSwitchSlowMode = true;

    public void switchSlowMode(boolean button) {
        if (!button) {
            firstPressSwitchSlowMode = true;
        }
        if (button && firstPressSwitchSlowMode) {
            firstPressSwitchSlowMode = false;
            slowMode = !slowMode;
        }
    }
}
