package org.firstinspires.ftc.teamcode;

/* CLASS SUMMARY:
 * Implements most of driver control within run(), which takes an input for side
 * This is done to reduce duplicate code between Red and Blue
 * */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AbstractVroom extends Control {
    boolean lastButtonX = false;
    boolean armWasManual = false;
    boolean wasDDR = false;

    double leftRamp = 0;
    double rightRamp = 0;

    ElapsedTime timeSinceDDR;
    ElapsedTime deltaTimer;

    public void driveLoop() {
        double left = ahi.drivetrainReverse ? -gamepad1.right_stick_y : -gamepad1.left_stick_y;
        double right = ahi.drivetrainReverse ? -gamepad1.left_stick_y : -gamepad1.right_stick_y;

        if(gamepad1.left_bumper || gamepad1.right_bumper) {
            left *= 0.6;
            right *= 0.6;

            double dt = deltaTimer.seconds();
            deltaTimer.reset();

            rightRamp += (right - rightRamp) * 4 * dt;
            leftRamp += (left - leftRamp) * 4 * dt;

            drive(leftRamp, rightRamp);
        } else {
            leftRamp = left;
            rightRamp = right;

            drive(left, right);
        }
    }

    public void run(FieldSide fieldSide) {
        waitForStart();

        final double fieldDir = (fieldSide == FieldSide.RED ? -1.0 : 1.0);
        timeSinceDDR = new ElapsedTime();
        deltaTimer = new ElapsedTime();

        while(opModeIsActive()) {
            driveLoop();

            if(gamepad2.right_bumper) {
                playDDR(fieldDir);
                timeSinceDDR.reset();
                wasDDR = true;
            } else if(gamepad2.left_bumper) {
                playDDR(-fieldDir);
                timeSinceDDR.reset();
                wasDDR = true;
            } else if(wasDDR) {
                if(ahi.ddr.getPower() != 0) {
                    playDDR(-ahi.ddr.getPower());
                }
                wasDDR = false;
            }

            if(timeSinceDDR.milliseconds() >= 60) {
                playDDR(0);
            }

            setFlup(gamepad2.left_trigger < .5);
            setLiftPower(-gamepad2.left_stick_y);

            runIntake((gamepad2.dpad_left ? 1 : 0) - gamepad2.right_trigger);

            if (gamepad2.x) {
                armControl(ArmPosition.PICKUP);
                lastButtonX = true;
            } else if (gamepad2.a) {
                armControl(ArmPosition.LOW);
                lastButtonX = false;
            } else if (gamepad2.b) {
                armControl(ArmPosition.MED);
                lastButtonX = false;
            } else if (gamepad2.y) {
                armControl(ArmPosition.HIGH);
                lastButtonX = false;
            } else if (lastButtonX) {
                armControl(ArmPosition.LOW_FORE);
                lastButtonX = false;
            }

            if (gamepad2.back) {
                telemetry.addLine("Reset");
                telemetry.update();
                ahi.arm.setPower(0);
                ahi.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                ahi.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ahi.arm.setPower(-gamepad2.left_stick_y);
                lastButtonX = false;
                armWasManual = true;
            } else if (armWasManual) {
                ahi.arm.setTargetPosition(ahi.arm.getCurrentPosition());
                ahi.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ahi.arm.setPower(1);
                armWasManual = false;
            }

            telemetry.addData("Arm Target", ahi.arm.getTargetPosition());
            telemetry.addData("Arm Current", ahi.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
