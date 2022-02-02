package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/* CLASS SUMMARY:
 * Contains functions to intuitively control various aspects of the robot
 * */

public abstract class Control extends LinearOpMode {
    public Ahi ahi;

    public enum FieldSide {
        RED,
        BLUE
    }

    public enum ArmPosition {
        START,
        LOW,
        LOW_FORE,
        MED,
        MED_FORE,
        HIGH,
        HIGH_FORE,
        UP,
        PICKUP,
    }

    final int armOffset = 0;
    final double CONVERSION_FACTOR = (1120/(43/29))/(5*Math.PI); // ticks per wheel rotation / wheel circumference

    public void initialize() {
        ahi = new Ahi(hardwareMap);
    }

    public void drive(double left, double right) {
        ahi.leftA.setPower(left);
        ahi.leftB.setPower(left);
        ahi.rightA.setPower(right);
        ahi.rightB.setPower(right);
    }

    public void armControl(ArmPosition armPosition) {
        switch(armPosition) { // 3360 ticks/rotation
            case LOW:
                ahi.arm.setTargetPosition(3833-armOffset);
                break;
            case LOW_FORE:
                ahi.arm.setTargetPosition(667-armOffset);
                break;
            case MED:
                ahi.arm.setTargetPosition(3333-armOffset);
                break;
            case MED_FORE:
                ahi.arm.setTargetPosition(1250-armOffset);
                break;
            case HIGH:
                ahi.arm.setTargetPosition(2500-armOffset); // 2583
                break;
            case HIGH_FORE:
                ahi.arm.setTargetPosition(1500-armOffset);
                break;
            case PICKUP:
                ahi.arm.setTargetPosition(0-armOffset);
                break;
            case UP:
                ahi.arm.setTargetPosition(1833-armOffset);
                break;
            default:
                ahi.arm.setTargetPosition(0);
                break;
        }
        ahi.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ahi.arm.setPower(1);
    }

    public void runIntake(double power) {
        ahi.succc.setPower(power);
    }

    public void playDDR(double power) {
        ahi.ddr.setPower(power*0.8);
    }

    public void setFlup(boolean open) {
        ahi.flup.setPosition(open ? 1.0 : 0.0);
    }

    public void setLiftPower(double power) {
        ahi.capLift.setPower(power);
    }

    public void zero() {
        drive(0, 0);
        runIntake(0);
        playDDR(0);
        setLiftPower(0);
        ahi.arm.setPower(0);
    }

    public boolean protectedSleep(long milliseconds) {
        for(long i = 0; i < milliseconds/10; i++) {
            sleep(10);
            if(isStopRequested()) {
                zero();
                return true;
            }
        }
        return false;
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        ahi.leftA.setMode(mode);
        ahi.leftB.setMode(mode);
        ahi.rightA.setMode(mode);
        ahi.rightB.setMode(mode);
    }
}
