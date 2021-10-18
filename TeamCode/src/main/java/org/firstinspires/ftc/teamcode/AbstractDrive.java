package org.firstinspires.ftc.teamcode;

public abstract class AbstractDrive extends Control {
    enum FieldSide {
        RED,
        BLUE
    }

    public void run(FieldSide fieldSide) throws InterruptedException {
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
