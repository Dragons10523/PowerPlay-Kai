package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control;

@TeleOp
public class DriveMecanum extends Control {

    @Override
    public void loop() {
        super.loop();
        double leftY = gamepad1.left_stick_y;// < .05 ? 0 : gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;// < .05 ? 0 : gamepad1.left_stick_x;
        double turn = gamepad1.left_trigger - gamepad1.right_trigger;
        SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();

        telemetry.addData("driveMode", driveMode);
        telemetry.addLine(String.format("XYH %6.2f %6.2f %6.2f", pose2D.x, pose2D.y, pose2D.h));

        mecanumDrive(leftY, leftX, turn);
        resetIMU(gamepad1.back);
        switchDriveMode(gamepad1.start);

    }
}