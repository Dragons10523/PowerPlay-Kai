package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Control extends OpMode {
    //TODO: Limiting switch on screw lift
    public RobotClass robot;
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

    public DriveMode driveMode = DriveMode.GLOBAL;
    public LiftMode liftMode = LiftMode.ENCODER_DRIVE;
    public LiftState liftState = LiftState.GROUND;
    public ArmState armState = ArmState.IN;

    public static enum FieldSide {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
        robot.initMotorsComp();
//        robot.initMotorsProto();
        robot.opticalSensor.calibrateImu();
    }
    @Override
    public void loop() {

    }
    public void mecanumDrive(double leftX, double leftY, double turn) {
        double heading;
        if (driveMode == DriveMode.GLOBAL) {
            heading = robot.getHeading();
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
    private boolean firstPress_Button_Gamepad_2_a = true;
    public void flipArm(boolean button_A){
        if(!button_A) firstPress_Button_Gamepad_2_a = true;

        if(button_A && firstPress_Button_Gamepad_2_a){
            firstPress_Button_Gamepad_2_a = false;
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
    public void resetIMU(boolean button) {
        if (!button) {
            return;
        }
        robot.resetIMU();
    }

    private boolean firstPress_startButtonGamepad_1 = true;
    public void switchDriveMode(boolean button) {

        if (!button) {
            firstPress_startButtonGamepad_1 = true;
        }
        if (button && firstPress_startButtonGamepad_1) {
            firstPress_startButtonGamepad_1 = false;
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
    public void manualArmFlip(double power){
        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(power);
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
    ElapsedTime time = new ElapsedTime();
    double timeAtPress = 0;
    public void flipBucket(boolean button){
        if(button){
            timeAtPress = time.seconds();
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0);
        }
        if(time.seconds() - 3 > timeAtPress && timeAtPress != 0){
            timeAtPress = 0;
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(.6);
        }
    }
    public void intakePower(double intakePower){
        intakePower = Math.min(.7, Math.max(-.7, intakePower));
        robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(intakePower);
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
