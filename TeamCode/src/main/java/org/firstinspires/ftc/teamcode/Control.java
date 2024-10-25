package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;

public class Control extends OpMode {
    public RobotClass robot;
    ElapsedTime elapsedTime = new ElapsedTime();
    public Utils utils;
    public SampleMecanumDrive drive;
    @Override
    public void init() {
        OpticalSensor opticalSensorObj = new OpticalSensor();
        robot = new RobotClass(hardwareMap);
        utils = new Utils(robot);
        robot.initMotorsComp();
        opticalSensorObj.configureOtos(robot);
        drive = new SampleMecanumDrive(hardwareMap);
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
