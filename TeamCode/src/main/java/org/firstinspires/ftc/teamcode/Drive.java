package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive")
public class Drive extends Control {
    // Assist values
    boolean assistDrive = true;
    boolean assistTurns = true;
    boolean assistManipulator = false;

    // Numerical values
    double assistTurnPower = 1;
    double assistDrivePower = 0.7;
    int selectedPole = 12;

    // Prev values
    boolean assistDrivePrev = false;
    boolean assistTurnPrev = false;
    boolean assistManipPrev = false;
    boolean clawPrev = false;
    boolean directionPrev = false;

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

            // Deadzone for alignment
            if(squaredHypotenuse(driveX, driveY) > 0.03) {
                double driveAngle = collapseAngle(Math.atan2(driveY, driveX) + kai.getHeading());
                int driveAlignment = ((int)(driveAngle / HALF_PI)) % 2;

                if(driveAlignment == 0) {
                    // X aligned
                    driveY += (12 - (kai.deadwheels.currentY % 24)) * assistDrivePower;
                } else {
                    // Y aligned
                    driveX += (12 - (kai.deadwheels.currentX % 24)) * assistDrivePower;
                }
            }
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

            if(!directionPrev) {
                if(gamepad2.dpad_up) {
                    selectedPole -= 5;
                } else if(gamepad2.dpad_down) {
                    selectedPole += 5;
                } else if(gamepad2.dpad_left) {
                    selectedPole -= 1;
                } else if(gamepad2.dpad_right) {
                    selectedPole += 1;
                }

                selectedPole %= 25;
            }

            directionPrev = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right;
            
            for(int i = 0; i < 5; i++) {
                StringBuilder line = new StringBuilder();
                for(int j = 0; j < 5; j++) {
                    int index = i * 5 + j;

                    if(index == selectedPole) {
                        line.append("*");
                        continue;
                    }

                    switch(FIELD_SETUP[index]) {
                        case GROUND:
                            line.append("G");
                            break;
                        case LOW:
                            line.append("L");
                            break;
                        case MID:
                            line.append("M");
                            break;
                        case HIGH:
                            line.append("H");
                            break;
                    }
                }
                telemetry.addLine(line.toString());
            }
        } else {
            claw = gamepad2.right_bumper;

            if(gamepad2.x) {
                lift(GoalHeight.INTAKE);
            } else if(gamepad2.y) {
                lift(GoalHeight.HIGH);
            } else if(gamepad2.a) {
                lift(GoalHeight.MID);
            } else if(gamepad2.b) {
                lift(GoalHeight.LOW);
            }
        }

        if(claw != clawPrev && claw) {
            toggleClaw();
        }

        telemetry.update();

        assistTurnPrev = gamepad1.a;
        assistDrivePrev = gamepad1.b;
        assistManipPrev = gamepad2.left_bumper;
        clawPrev = claw;
    }
}