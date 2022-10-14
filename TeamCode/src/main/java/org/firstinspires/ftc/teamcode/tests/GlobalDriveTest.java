package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control;

@TeleOp(name = "Global", group = "Test")
public class GlobalDriveTest extends Control {
    @Override
    public void loop() {
        super.loop();

        mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_trigger - gamepad1.left_trigger, DriveMode.GLOBAL);
        telemetry.addData("X", kai.deadwheels.currentX);
        telemetry.addData("Y", kai.deadwheels.currentY);
        telemetry.addData("θ", kai.deadwheels.currentAngle);
        telemetry.update();
    }
}
