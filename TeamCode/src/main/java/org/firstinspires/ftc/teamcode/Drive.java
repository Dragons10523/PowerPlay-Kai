package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive")
public class Drive extends Control {
    // Assist values
    boolean assistDrive = true;
    boolean assistTurns = true;
    boolean assistManipulator = true;

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

    // Other
    boolean wantToLower = false;

    @Override
    public void loop() {
        super.loop();

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
            kai.deadwheels.setTransform(0, 0, 0);
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

        boolean manualClaw = gamepad2.right_bumper;
        if(assistManipulator) {
            // Pole Selection
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

            boolean clawOpen = clawOpen();
            if(clawOpen && gamepad2.x && clawDistance() <= 1.5) {
                clawOpen = false;
            }
            if(!clawOpen && gamepad2.x && willConeHit(selectedPole)) {
                clawOpen = true;
            }

            if(clawOpen != clawOpen()) {
                toggleClaw();
            }

            directionPrev = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right;

            // TODO: Replace 350 with the wall height
            if(kai.armLiftA.getTargetPosition() > 350 && kai.armLiftA.getCurrentPosition() > 350 && !wantToLower) {
                aimClaw(selectedPole);
            } else {
                aimClaw(0);
            }

            displayField();

            // Raising the claw
            GoalHeight poleHeight = FIELD_SETUP[selectedPole];

            wantToLower = false;
            if(Math.abs(tableAngle() % Math.PI) <= 0.06) {
                lift(poleHeight);
            } else {
                switch(poleHeight) {
                    case GROUND:
                    case LOW:
                    case NONE:
                        aimClaw(0);
                        wantToLower = true;
                    default:
                        lift(poleHeight);
                }
            }
        } else {
            if(gamepad2.x) {
                lift(GoalHeight.NONE);
            } else if(gamepad2.y) {
                lift(GoalHeight.HIGH);
            } else if(gamepad2.a) {
                lift(GoalHeight.MID);
            } else if(gamepad2.b) {
                lift(GoalHeight.LOW);
            }

            if(manualClaw != clawPrev && manualClaw) {
                toggleClaw();
            }
        }

        telemetry.update();

        assistTurnPrev = gamepad1.a;
        assistDrivePrev = gamepad1.b;
        assistManipPrev = gamepad2.left_bumper;
        clawPrev = manualClaw;
    }

    public void displayField() {
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
                    default:
                        line.append(" ");
                }
            }

            telemetry.addLine(line.toString());
        }
    }
}