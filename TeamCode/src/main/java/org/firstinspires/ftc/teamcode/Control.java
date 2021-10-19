package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/* CLASS SUMMARY:
 * Contains functions to intuitively control various aspects of the robot
 * */

public abstract class Control extends LinearOpMode {
    Ahi ahi;

    enum FieldSide {
        RED,
        BLUE
    }

    final int armOffset = 0;
    final double CONVERSION_FACTOR = 8192*(4.0+(2.0/7.0)*Math.PI);

    enum ArmPosition {
        START,
        LOW,
        MED,
        HIGH,
        PICKUP
    }

    public void drive(double left, double right) {
        ahi.leftA.setPower(left);
        ahi.leftB.setPower(left);
        ahi.rightA.setPower(right);
        ahi.rightB.setPower(right);
    }

    public void armControl(ArmPosition armPosition) {
        switch(armPosition) {
            case LOW:
                ahi.arm.setTargetPosition(3640-armOffset);
                break;
            case MED:
                ahi.arm.setTargetPosition(3300-armOffset);
                break;
            case HIGH:
                ahi.arm.setTargetPosition(2960-armOffset);
                break;
            case PICKUP:
                ahi.arm.setTargetPosition(0-armOffset);
                break;
            default:
                ahi.arm.setTargetPosition(0);
                break;
        }
        ahi.arm.setPower(1.0);
    }

    public void runIntake(boolean on) {
        ahi.succc.setPower(on ? 1.0 : 0.0);
    }

    public void playDDR(double power) {
        ahi.ddr.setPower(power);
    }

    public void setFlup(boolean open) {
        ahi.flup.setPosition(open ? 1.0 : 0.0);
    }

    public void setLiftPower(double power) {
        ahi.capLift.setPower(power);
    }
}
