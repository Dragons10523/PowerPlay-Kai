package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Control extends LinearOpMode {
    Ahi ahi;

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

    public void driveLoop() {
        double left = ahi.drivetrainReverse ? -gamepad1.right_stick_y : -gamepad1.left_stick_y;
        double right = ahi.drivetrainReverse ? -gamepad1.left_stick_y : -gamepad1.right_stick_y;

        boolean sneak = gamepad1.left_stick_button || gamepad1.right_stick_button;

        left *= sneak ? 0.3 : 1.0;
        right *= sneak ? 0.3 : 1.0;

        drive(left, right);
    }

    public void driveDist(double dist) {
        int ticks = (int)(dist*CONVERSION_FACTOR);
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

    public void runIntake(double power) {
        ahi.succc.setPower(power);
    }

    public void playDDR(double power) {
        ahi.ddr.setPower(power);
    }

    public void setFlup(boolean open) {
        ahi.flup.setPosition(open ? 1.0 : 0.0);
    }

    public void setLiftHeight(double power) {
        ahi.capLift.setPower(power);
    }
}
