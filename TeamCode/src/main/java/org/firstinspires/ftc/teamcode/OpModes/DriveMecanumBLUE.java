package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.Arrays;
import java.util.Locale;

@TeleOp
public class DriveMecanumBLUE extends Control {
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
        float[] hsvValues = colorSensorClassObj.getHsvValues();

        double liftPower = gamepad2.dpad_down ? -1 : 0;
        liftPower += gamepad2.dpad_up ? 1 : 0;
        double armPower = gamepad2.left_trigger - gamepad2.right_trigger;

        SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();

        telemetry.addData("driveMode", Utils.driveMode);
        telemetry.addData("liftMode", Utils.liftMode);
        telemetry.addData("liftState", Utils.liftState);
        telemetry.addData("armState", Utils.armState);
        telemetry.addData("heading", robot.getHeading());
        telemetry.addData("hsvValues", Arrays.toString(hsvValues));
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", robot.colorSensor.alpha());
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("currentVoltage", robot.voltageSensor.getVoltage());
        telemetry.update();

        if(Utils.slowMode) utils.mecanumDrive(leftY/2, leftX/2, turn/2);
        else utils.mecanumDrive(leftY, leftX, turn);

        utils.resetIMU(gamepad1.back);
        utils.liftPower(liftPower, gamepad2.a);
        utils.resetLift(gamepad2.back);
        utils.switchDriveMode(gamepad1.start);
        utils.switchLiftMode(gamepad2.start);
        utils.extendAndRetractArm(-gamepad2.right_stick_y);
        utils.flipArm(armPower, gamepad2.y);
        utils.flipBucket(gamepad2.b);
        utils.intakeServo(gamepad2.x);
        if(robot.distanceSensor.getDistance(DistanceUnit.CM) < 5){
            if(robot.colorSensor.red() > robot.colorSensor.blue() && robot.colorSensor.alpha() > 160){
                utils.powerIntake(1);
            }
        }
        else{
            utils.powerIntake(gamepad2.left_stick_y);
        }
        utils.switchSlowMode(gamepad1.a);
        utils.deathWiggle(gamepad1.b);
    }
}