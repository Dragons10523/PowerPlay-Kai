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

    public void driveLoop() {
        double left = ahi.drivetrainReverse ? -gamepad1.right_stick_y : -gamepad1.left_stick_y;
        double right = ahi.drivetrainReverse ? -gamepad1.left_stick_y : -gamepad1.right_stick_y;

        boolean sneak = gamepad1.left_bumper || gamepad1.right_bumper;

        double speed = (left + right) / 2;
        double turn = (left - right) / 2;

        speed *= sneak ? 0.6 : 1.0;

        if(ahi.arm.isBusy()) {
            turn *= .4;
        }

        drive(speed + turn, speed - turn);
    }

    public void run(FieldSide fieldSide) {
        waitForStart();

        final double fieldDir = (fieldSide == FieldSide.RED ? -1.0 : 1.0);

        while(opModeIsActive()) {
            driveLoop();

            playDDR(gamepad2.right_bumper ? fieldDir : 0.0);
            playDDR(gamepad2.left_bumper ? -fieldDir : 0.0);
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
                ahi.arm.setPower(0);
                armWasManual = false;
            }

            telemetry.addData("Arm Target", ahi.arm.getTargetPosition());
            telemetry.addData("Arm Current", ahi.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
