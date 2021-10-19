package org.firstinspires.ftc.teamcode;

/* CLASS SUMMARY:
 * Implements most of driver control within run(), which takes an input for side
 * This is done to reduce duplicate code between Red and Blue
 * */

public abstract class AbstractDrive extends Control {
    public void driveLoop() {
        double left = ahi.drivetrainReverse ? -gamepad1.right_stick_y : -gamepad1.left_stick_y;
        double right = ahi.drivetrainReverse ? -gamepad1.left_stick_y : -gamepad1.right_stick_y;

        boolean sneak = gamepad1.left_stick_button || gamepad1.right_stick_button;

        left *= sneak ? 0.3 : 1.0;
        right *= sneak ? 0.3 : 1.0;

        drive(left, right);
    }

    public void run(FieldSide fieldSide) {
        waitForStart();

        final double fieldDir = (fieldSide == FieldSide.RED ? 1.0 : -1.0);

        while(opModeIsActive()) {
            driveLoop();

            playDDR(gamepad2.right_bumper ? fieldDir : 0.0);
            setFlup(gamepad2.left_trigger > .5);
            runIntake(gamepad2.right_trigger > .5);
            setLiftPower(-gamepad2.left_stick_y);

            if (gamepad2.x) {
                armControl(ArmPosition.PICKUP);
            } else if (gamepad2.a) {
                armControl(ArmPosition.LOW);
            } else if (gamepad2.b) {
                armControl(ArmPosition.MED);
            } else if (gamepad2.y) {
                armControl(ArmPosition.HIGH);
            }
        }
    }
}
