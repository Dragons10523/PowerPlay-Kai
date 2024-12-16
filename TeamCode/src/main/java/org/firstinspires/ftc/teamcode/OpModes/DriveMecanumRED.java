package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.Arrays;
import java.util.Locale;

@TeleOp
public class DriveMecanumRED extends Control {
    @Override
    public void start() {
        robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.79);
        robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.08);
        robot.Motors.get(RobotClass.MOTORS.LIFT).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        super.loop();
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double turn = gamepad1.left_trigger - gamepad1.right_trigger;
        float[] hsvValues = colorSensorClassObj.getHsvValues();

        double liftPower = gamepad2.dpad_down ? -1 : 0;
        liftPower += gamepad2.dpad_up ? 1 : 0;
        double armPower = gamepad2.left_trigger - gamepad2.right_trigger;

        telemetry.addData("driveMode", Utils.driveMode);
        telemetry.addData("liftMode", Utils.liftMode);
        telemetry.addData("armState", Utils.armState);
        telemetry.addData("heading", robot.getHeading());
        telemetry.addData("liftPos", robot.Motors.get(RobotClass.MOTORS.LIFT).getCurrentPosition());
        telemetry.addData("armPos", robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition());
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("currentVoltage", robot.voltageSensor.getVoltage());
        telemetry.update();

        if (Utils.slowMode) utils.mecanumDrive(leftY / 2, leftX / 2, turn / 2);
        else utils.mecanumDrive(leftY, leftX, turn);

        utils.resetIMU(gamepad1.back);
        if(Utils.liftMode == Utils.LiftMode.LIFT) utils.liftPower(liftPower, gamepad2.a);
        else utils.hangLiftPower(liftPower);
        utils.switchLiftMode(gamepad2.start);
        utils.switchDriveMode(gamepad1.start);
        utils.extendAndRetractArm(-gamepad2.right_stick_y);
        utils.flipArm(armPower, gamepad2.y);
        utils.flipBucket(gamepad2.b);
        utils.intakeServo(gamepad2.x);
        utils.specimenGrab(gamepad2.left_bumper);
        utils.powerIntake(gamepad2.left_stick_y);
//        if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 5) {
//            if (robot.colorSensor.blue() > robot.colorSensor.red() && robot.colorSensor.alpha() > 160 && robot.colorSensor.blue() > 1000) {
//                utils.powerIntake(1);
//            }
//        } else {
//            utils.powerIntake(gamepad2.left_stick_y);
//        }
        utils.switchSlowMode(gamepad1.a);
        utils.deathWiggle(gamepad1.b);
    }
}