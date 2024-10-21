package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;

import java.util.HashMap;
import java.util.Map;

@TeleOp
public class TestOpMode extends Control {
    Map<RobotClass.MOTORS, Double> wheelSpeeds = new HashMap<>();
    boolean atTarget = false;
    double angularDistance = 0;
    double targetAngle = -90.0;
    int turnVal = 1;

    @Override
    public void start() {
        super.start();
    }
    ElapsedTime time = new ElapsedTime();
    @Override
    public void loop() {
        super.loop();
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(gamepad1.left_trigger - gamepad1.right_trigger);
    }

    public void UpdateWheelPowers() {
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT));
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT));
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT));
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT));
    }

}
