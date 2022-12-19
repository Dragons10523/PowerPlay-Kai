package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.VecUtils;

@TeleOp(name = "Drive")
public class Drive extends Control {
    public final boolean INTENSIVE_PRACTICE = false;

    // Assist values
    boolean assistDrive = false;
    boolean assistTurns = true;
    boolean assistManipulator = false;

    // Numerical values
    double assistTurnPower = 1.6;
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

    // Intensive driver practice variables
    ElapsedTime practiceTimer = new ElapsedTime();
    double timeUntilEvent = -1;
    boolean clawSensorBroken = false;

    @Override
    public void start() {
        super.start();
    }

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
            kai.deadwheels.setTransform(0, 0, 0);
        }

        // Turning
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        if(assistTurns) {// && Math.abs(turn) <= .1) {
            telemetry.addData("Angle", (int) Math.toDegrees(kai.getHeading()));
            double addend = -mapAngle(kai.getHeading(), -VecUtils.HALF_PI/2, VecUtils.HALF_PI/2, 0);
            turn += addend * addend * assistTurnPower;
        }

        telemetry.addData("Turn", turn);

        // Driving
        float driveX = gamepad1.left_stick_x;
        float driveY = -gamepad1.left_stick_y;
        DriveMode driveMode = DriveMode.LOCAL;

        if(assistDrive) {
            driveMode = DriveMode.GLOBAL;

            // Deadzone for alignment
            if(squaredHypotenuse(driveX, driveY) > 0.03) {
                double driveAngle = collapseAngle(Math.atan2(driveY, driveX) + kai.getHeading());
                int driveAlignment = ((int)(driveAngle / VecUtils.HALF_PI)) % 2;

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

            armControl.setTarget(Control.poleIdxToPos(selectedPole));

            boolean clawOpen = armControl.isClawOpen();
            if(clawOpen && gamepad2.x && clawDistance() <= 1.5 && !clawSensorBroken) {
                clawOpen = false;
            }
            if(!clawOpen && gamepad2.x && armControl.willConeHit(selectedPole)) {
                clawOpen = true;
            }

            if(clawOpen != armControl.isClawOpen()) {
                armControl.toggleClaw();
            }

            directionPrev = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right;

            orientClaw(WristState.NORMAL);

            displayField();

            // Raising the claw
            GoalHeight poleHeight = FIELD_SETUP[selectedPole];
        } else {
            if(gamepad2.x) {
                armControl.setLiftHeight(GoalHeight.NONE);
            } else if(gamepad2.y) {
                armControl.setLiftHeight(GoalHeight.HIGH);
            } else if(gamepad2.a) {
                armControl.setLiftHeight(GoalHeight.MID);
            } else if(gamepad2.b) {
                armControl.setLiftHeight(GoalHeight.LOW);
            }

            if(manualClaw != clawPrev && manualClaw) {
                armControl.toggleClaw();
            }

            if(gamepad2.dpad_down) {
                orientClaw(WristState.FLIPPED);
            } else if(gamepad2.dpad_up) {
                orientClaw(WristState.NORMAL);
            }
        }

        // Intensive Practice Things
        if(INTENSIVE_PRACTICE && practiceTimer.seconds() >= timeUntilEvent) {
            practiceTimer.reset();

            if(timeUntilEvent != -1) {
                // random value 0-5 inclusive
                int eventValue = (int) Math.floor(Math.random() * 5);

                DcMotor[] liftMotors = {kai.armLiftA, kai.armLiftB};
                switch(eventValue) {
                    // Randomly set motors to float
                    case 0:
                        for(DcMotor motor : kai.drivetrain.driveMotors) {
                            int willBreak = (int) Math.floor(Math.random() * 2);
                            if(willBreak == 0) {
                                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                            }
                        }
                        break;
                    // Randomly set motors and encoders to break
                    case 1:
                        for(DcMotor motor : kai.drivetrain.driveMotors) {
                            int willBreak = (int) Math.floor(Math.random() * 4);
                            if(willBreak == 0) {
                                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            }
                        }
                        break;
                    // Randomly break one of the lift motors
                    case 2:
                        for(DcMotor motor : liftMotors) {
                            int willBreak = (int) Math.floor(Math.random() * 3);
                            if(willBreak == 0) {
                                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            }
                        }
                        break;
                    // Randomly poke the deadwheels
                    case 3:
                        int willBreak = (int) Math.floor(Math.random() * 2);

                        if(willBreak == 0) {
                            kai.deadwheels.currentAngle += .1*(Math.random()-.5);
                        }

                        kai.deadwheels.currentX += 2*(Math.random()-.5);
                        kai.deadwheels.currentY += 2*(Math.random()-.5);
                        break;
                    // Break the claw sensor
                    case 4:
                        clawSensorBroken = true;
                        break;
                }
            }

            // Random value between 10 and 120
            timeUntilEvent = Math.random() * 130 + 10;
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