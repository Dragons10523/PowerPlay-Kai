package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;

public class Control extends OpMode {
    public RobotClass robot;
    double timeWhenPressed = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
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
    public static enum FieldSide {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public DriveMode driveMode = DriveMode.GLOBAL;
    public LiftMode liftMode = LiftMode.ENCODER_DRIVE;
    public LiftState liftState = LiftState.GROUND;
    public ArmState armState = ArmState.IN;
    double bucketStartPos;

    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
        robot.initMotorsProto();
        OpticalSensor.configureOtos(robot);
        robot.opticalSensor.resetTracking();
       // bucketStartPos = robot.Servos.get(RobotClass.SERVOS.BUCKET).getPosition();
    }
    @Override
    public void loop() {

    }
    public void mecanumDrive(double leftX, double leftY, double turn) {

        double heading;
        if (driveMode == DriveMode.GLOBAL) {
            heading = robot.opticalSensor.getPosition().h;
        } else {
            heading = 0;
        }

        robot.drivetrain.mecanumDrive(leftY, leftX, turn, heading, telemetry);
    }
    boolean firstPressDPAD = true;
    public void liftPower(double liftPower){
        if(liftMode == LiftMode.ENCODER_DRIVE){
            if(liftPower == 0){
                firstPressDPAD = true;
            }
            if(liftPower == 1 && firstPressDPAD){
                firstPressDPAD = false;
                switch(liftState) {
                    case GROUND:
                        liftState = LiftState.LOW;
                        break;
                    case LOW:
                        liftState = LiftState.HIGH;
                        break;
                }
                runToLiftPos();
            }
            if(liftPower == -1 && firstPressDPAD){
                firstPressDPAD = false;
                switch(liftState){
                    case HIGH:
                        liftState = LiftState.LOW;
                        break;
                    case LOW:
                        liftState = LiftState.GROUND;
                        break;
                }
                runToLiftPos();
            }
        }else{
            robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setPower(liftPower);
            robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setPower(liftPower);
        }
    }
    private boolean firstPress_Button_A = true;
    public void extendAndRetractArm(boolean button_A){
        if(!button_A) firstPress_Button_A = true;

        if(button_A && firstPress_Button_A){
            firstPress_Button_A = false;
            switch(armState){
                case IN:
                    armState = ArmState.EXTENDED;
                    break;
                case EXTENDED:
                    armState = ArmState.IN;
                    break;
            }
            runToArmState();
        }

    }
    public void flipArm(double power){
        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMotorEnable();

        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(power);
    }
    public void resetIMU(boolean button) {
        if (!button) {
            return;
        }
        robot.resetIMU();
    }
    private boolean firstPress_startButton_Gamepad_1 = true;
    public void switchDriveMode(boolean button) {

        if (!button) {
            firstPress_startButton_Gamepad_1 = true;
        }
        if (button && firstPress_startButton_Gamepad_1) {
            firstPress_startButton_Gamepad_1 = false;
            switch (driveMode) {
                case GLOBAL:
                    driveMode = DriveMode.LOCAL;
                    break;
                case LOCAL:
                    driveMode = DriveMode.GLOBAL;
                    break;
            }
        }
    }
    private boolean firstPress_startButton_Gamepad_2 = true;
    public void switchLiftMode(boolean button){
        if (!button) {
            firstPress_startButton_Gamepad_2 = true;
        }
        if (button && firstPress_startButton_Gamepad_2) {
            firstPress_startButton_Gamepad_2 = false;
            switch (liftMode) {
                case ENCODER_DRIVE:
                    liftMode = LiftMode.RAW_POWER;
                    robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    break;
                case RAW_POWER:
                    liftMode = LiftMode.ENCODER_DRIVE;
                    robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    break;
            }
        }
    }
    public void runToLiftPos(){
        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMotorEnable();
        robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMotorEnable();

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
    public void runToArmState(){
        switch(armState){
            case IN:
                robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0);
                robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0);
            case EXTENDED:
                robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(-1);
                robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(-1);
        }
    }
    public void disableNonBusyMotors(){
        if(!robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).isBusy()){
            robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).setMotorDisable();
        }
        if(robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).getPower() != 0){
            robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).setMotorDisable();
        }
        if(robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getPower() != 0){
            robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMotorDisable();
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
    boolean B_2_isFirstPress = true;
    public void flipBucket(boolean button){
        if(!button){
            B_2_isFirstPress = true;
        }
        if(button && B_2_isFirstPress){
            B_2_isFirstPress = false;
            timeWhenPressed = elapsedTime.seconds();
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0);
        }
        if(elapsedTime.seconds() - 3 > timeWhenPressed ){
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(1);
        }
    }
    @Override
    public void stop() {
        if (robot == null) return; // ensures that stop() is not called before initialization
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0);
    }
}
