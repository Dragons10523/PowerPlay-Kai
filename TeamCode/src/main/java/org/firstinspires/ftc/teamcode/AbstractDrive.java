package org.firstinspires.ftc.teamcode;

/* CLASS SUMMARY:
 * Implements most of driver control within run(), which takes an input for side
 * This is done to reduce duplicate code between Red and Blue
 * */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AbstractDrive extends Control {
    boolean isPickup = false;
    ElapsedTime elapsedTime;

    public void driveLoop() {
        double left = ahi.drivetrainReverse ? -gamepad1.right_stick_y : -gamepad1.left_stick_y;
        double right = ahi.drivetrainReverse ? -gamepad1.left_stick_y : -gamepad1.right_stick_y;

        boolean sneak = gamepad1.left_bumper || gamepad1.right_bumper;

        left *= sneak ? 0.6 : 1.0;
        right *= sneak ? 0.6 : 1.0;

        drive(left, right);
    }

    public void run(FieldSide fieldSide) {
        waitForStart();

        elapsedTime = new ElapsedTime();

        final double fieldDir = (fieldSide == FieldSide.RED ? -1.0 : 1.0);

        while(opModeIsActive()) {
            driveLoop();

            playDDR(gamepad2.right_bumper ? fieldDir : 0.0);
            playDDR(gamepad2.left_bumper ? -fieldDir : 0.0);
            setFlup(gamepad2.left_trigger < .5);
            runIntake((gamepad2.dpad_left ? 1 : 0) - gamepad2.right_trigger);
            setLiftPower(-gamepad2.left_stick_y);

            if (gamepad2.x) {
                armControl(ArmPosition.MED_FORE);
                isPickup = true;
            } else if (gamepad2.a) {
                armControl(ArmPosition.LOW);
                isPickup = false;
            } else if (gamepad2.b) {
                armControl(ArmPosition.MED);
                isPickup = false;
            } else if (gamepad2.y) {
                armControl(ArmPosition.HIGH);
                isPickup = false;
            }

            if(isPickup && ahi.arm.getCurrentPosition() <= 650) {
            } else {
                elapsedTime.reset();
            }

            if(isPickup && elapsedTime.milliseconds() > 500) {
                armControl(ArmPosition.PICKUP);
            }

            if (gamepad2.dpad_up) {
                ahi.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                ahi.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ahi.arm.setPower(-gamepad2.left_stick_y);
            } else {
                ahi.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("Arm Target", ahi.arm.getTargetPosition());
            telemetry.addData("Arm Current", ahi.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
