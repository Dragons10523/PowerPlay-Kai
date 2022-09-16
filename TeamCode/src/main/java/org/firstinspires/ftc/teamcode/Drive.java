package org.firstinspires.ftc.teamcode;

public class Drive extends Control {
    // Assist values
    boolean assistDrive = true;
    boolean assistTurns = true;
    boolean assistManipulator = false;

    // Numerical values
    double assistTurnPower = 1;

    // Prev values
    boolean assistDrivePrev = false;
    boolean assistTurnPrev = false;
    boolean assistManipPrev = false;
    boolean clawPrev = false;

    @Override
    public void loop() {
        // GAMEPAD1
        // Assist Toggles
        if(gamepad1.a != assistTurnPrev && gamepad1.a) {
            assistDrive = !assistDrive;
        }
        if(gamepad1.b != assistDrivePrev && gamepad1.b) {
            assistDrive = !assistDrive;
        }

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

        // GAMEPAD2
        if(gamepad2.left_bumper != assistManipPrev && gamepad2.left_bumper) {
            assistManipulator = !assistManipulator;
        }

        // TODO: claw aim and release
        boolean claw;
        if(assistManipulator) {
            claw = gamepad2.x;
        } else {
            claw = gamepad2.right_bumper;

            if(gamepad2.x) {
                lift(LiftHeight.INTAKE);
            } else if(gamepad2.y) {
                lift(LiftHeight.HIGH);
            } else if(gamepad2.a) {
                lift(LiftHeight.MID);
            } else if(gamepad2.b) {
                lift(LiftHeight.LOW);
            }
        }

        if(claw != clawPrev && claw) {
            toggleClaw();
        }

        assistTurnPrev = gamepad1.a;
        assistDrivePrev = gamepad1.b;
        assistManipPrev = gamepad2.left_bumper;
        clawPrev = claw;
    }
}