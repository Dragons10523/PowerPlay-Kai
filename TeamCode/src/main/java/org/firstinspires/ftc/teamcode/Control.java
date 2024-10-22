package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;

public class Control extends OpMode {
    public RobotClass robot;
    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
        robot.initMotorsProto();
        OpticalSensor.configureOtos(robot);
        robot.opticalSensor.resetTracking();
    }
    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        if (robot == null) return; // ensures that stop() is not called before initialization
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0);
    }
}
