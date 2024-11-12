package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utils;

@TeleOp
public class DriveMecanum extends Control {
    @Override
    public void start() {
        robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.67);
        robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.52);
    }
    @Override
    public void loop() {
        super.loop();
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double turn = gamepad1.left_trigger - gamepad1.right_trigger;

        double liftPower = gamepad2.dpad_down ? -1 : 0;
        liftPower += gamepad2.dpad_up ? 1 : 0;
        double intakePower = gamepad2.left_stick_y;
        double armPower = gamepad2.left_trigger - gamepad2.right_trigger;

        SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();

        telemetry.addData("driveMode", Utils.driveMode);
        telemetry.addData("liftMode", Utils.liftMode);
        telemetry.addData("liftPos", Utils.liftState);
        telemetry.addData("armState", Utils.armState);
        telemetry.addData("heading", robot.getHeading());
        telemetry.addData("armPos", robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition());
        telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
        telemetry.addData("currentVoltage", robot.voltageSensor.getVoltage());
        telemetry.update();

        if(Utils.slowMode) utils.mecanumDrive(leftY/2, leftX/2, turn/2);
        else utils.mecanumDrive(leftY, leftX, turn);

        utils.resetIMU(gamepad1.back);
        utils.liftPower(liftPower);
        utils.switchDriveMode(gamepad1.start);
        utils.switchLiftMode(gamepad2.start);
        utils.extendAndRetractArm(-gamepad2.right_stick_y);
        utils.flipArm(armPower, gamepad2.y);
        utils.flipBucket(gamepad2.b);
        utils.powerIntake(intakePower);
        utils.switchSlowMode(gamepad1.a);
        utils.deathWiggle(gamepad1.b);
    }
}