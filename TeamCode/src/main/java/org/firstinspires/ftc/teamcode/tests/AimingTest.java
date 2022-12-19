package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.processors.Control;

@TeleOp(name = "Aiming", group = "Test")
public class AimingTest extends Control {
    int selectedPole = 12;
    boolean directionPrev = false;

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        if(!directionPrev) {
            if(gamepad1.dpad_up) {
                selectedPole -= 5;
            } else if(gamepad1.dpad_down) {
                selectedPole += 5;
            } else if(gamepad1.dpad_left) {
                selectedPole -= 1;
            } else if(gamepad1.dpad_right) {
                selectedPole += 1;
            }
            selectedPole %= 25;
        }

        directionPrev = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;

        armControl.setTarget(Control.poleIdxToPos(selectedPole));

        displayField();
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
