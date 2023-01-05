package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.processors.Control;

@TeleOp(name = "Deadwheel", group = "Test")
public class DeadwheelTest extends Control {
    @Override
    public void loop() {
        kai.deadwheels.wheelLoop();

        mecanumDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_trigger - gamepad1.left_trigger, DriveMode.LOCAL);

        //telemetry.addData("0", kai.drivetrain.driveMotors[0].getCurrentPosition());
        //telemetry.addData("1", kai.drivetrain.driveMotors[1].getCurrentPosition());
        //telemetry.addData("2", kai.drivetrain.driveMotors[2].getCurrentPosition());
        //telemetry.addData("3", kai.drivetrain.driveMotors[3].getCurrentPosition());
        telemetry.addData("X", kai.deadwheels.currentX);
        telemetry.addData("Y", kai.deadwheels.currentY);
        telemetry.addData("θ", kai.deadwheels.currentAngle);
        telemetry.update();
    }
}
