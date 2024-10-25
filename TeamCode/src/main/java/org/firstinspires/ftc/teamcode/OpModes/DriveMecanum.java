package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utils;

@TeleOp
public class DriveMecanum extends Control {


    @Override
    public void loop() {
        super.loop();
        double leftY = gamepad1.left_stick_y;// < .05 ? 0 : gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;// < .05 ? 0 : gamepad1.left_stick_x;
        double turn = gamepad1.left_trigger - gamepad1.right_trigger;
        boolean A_2 = gamepad2.a;
        boolean B_2 = gamepad2.b;

        double liftPower = gamepad2.dpad_down ? -1 : 0;
        liftPower += gamepad2.dpad_up ? 1 : 0;
        double intakePower = gamepad2.left_stick_y;
        double armPower = gamepad2.left_trigger - gamepad2.right_trigger;


        SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();

        //disableNonBusyMotors();
        //^^ causes stuttering in motors
        telemetry.addData("driveMode", Utils.driveMode);
        telemetry.addData("liftMode", Utils.liftMode);
        telemetry.addData("liftPos", Utils.liftState);
        telemetry.addData("LeftLiftEncoder", robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).getCurrentPosition());
        telemetry.addData("RightLiftEncoder", robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).getCurrentPosition());
        telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
        telemetry.addData("currentDrawLiftMotors_MILLIAMPS",
                robot.Motors.get(RobotClass.MOTORS.LIFT_LEFT).getCurrent(CurrentUnit.MILLIAMPS) + robot.Motors.get(RobotClass.MOTORS.LIFT_RIGHT).getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("currentDrawDriveTrain_MILLIAMPS", utils.getCurrentDrawDriveTrain(CurrentUnit.MILLIAMPS));
        telemetry.addData("currentVoltage", robot.voltageSensor.getVoltage());
        double leftPos = robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).getPosition();
        double rightPos = robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).getPosition();

        telemetry.addData("leftPos", leftPos);
        telemetry.addData("rightPos", rightPos);
        telemetry.addData("armState", Utils.armState);
        telemetry.update();
        //drive.setDrivePower(new Pose2d(leftY, leftX, turn));
        utils.mecanumDrive(leftY, leftX, turn);
        utils.resetIMU(gamepad1.back);
        utils.liftPower(liftPower);
        utils.switchDriveMode(gamepad1.start);
        utils.switchLiftMode(gamepad2.start);
        utils.extendAndRetractArm(A_2);
        utils.flipArm(armPower);
        utils.flipBucket(B_2);
        utils.powerIntake(intakePower);
    }
}