package org.firstinspires.ftc.teamcode;

public class Drive extends Control {
    boolean assistDrive = true;
    boolean assistTurns = true;

    double assistTurnPower = 1;

    boolean aPrev = false;
    boolean bPrev = false;

    @Override
    public void loop() {
        // GAMEPAD1
        // Assist Toggles
        if(gamepad1.a != aPrev && gamepad1.a) {
            assistDrive = !assistDrive;
        }
        if(gamepad1.b != bPrev && gamepad1.b) {
            assistDrive = !assistDrive;
        }

        aPrev = gamepad1.a;
        bPrev = gamepad1.b;

        // Calibration
        if(gamepad1.back) {
            resetHeading();
        }

        // Turning
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        if(assistTurns) {
            turn += assistTurnPower * mapAngle(kai.getHeading(), -HALF_PI, HALF_PI, 0);
        }

        // Driving
        double driveX = gamepad1.left_stick_x;
        double driveY = -gamepad1.left_stick_y;
        DriveMode driveMode = DriveMode.LOCAL;

        if(assistDrive) {
            driveMode = DriveMode.GLOBAL;

            // TODO add auto alignment once dead wheels have been added
            // Deadzone for alignment
            /*if(squaredHypotenuse(driveX, driveY) > 0.03) {
                double driveAngle = collapseAngle(Math.atan2(driveY, driveX) + kai.getHeading());
                int driveAlignment = ((int)(driveAngle / HALF_PI)) % 2;

                if(driveAlignment == 0) {
                    // X aligned
                } else {
                    // Y aligned
                }
            }*/
        }

        mecanumDrive(driveX, driveY, turn, driveMode);
    }
}