package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.processors.AutoControl;
import org.firstinspires.ftc.teamcode.processors.VecUtils;

@TeleOp(name = "ConeTest", group = "Test")
public class ConeStackTest extends AutoControl {
    double assistTurnPower = 1;
    double assistDrivePower = 0.7;

    boolean heightChanged = false;
    int coneStackHeight = 5;

    @Override
    public void loop() {
        super.loop();

        float driveX = gamepad1.left_stick_x;
        float driveY = -gamepad1.left_stick_y;
        DriveMode driveMode = DriveMode.LOCAL;

        driveMode = DriveMode.GLOBAL;

        // Turning
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        turn += assistTurnPower * mapAngle(kai.getHeading(), -VecUtils.HALF_PI, VecUtils.HALF_PI, 0);

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

        mecanumDrive(driveX, driveY, turn, driveMode);

        if(!heightChanged) {
            if(gamepad1.dpad_down) {
                coneStackHeight--;
            } else if(gamepad1.dpad_up) {
                coneStackHeight++;
            }
        }
        heightChanged = gamepad1.dpad_down || gamepad1.dpad_up;

        coneStackHeight %= 5;

        armControl.coneStack = coneStackHeight;
    }
}
