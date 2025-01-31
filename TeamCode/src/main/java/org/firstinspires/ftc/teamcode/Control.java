package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Auto.ColorSensorClass;
import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;

public class Control extends OpMode {
    public RobotClass robot;
    public Utils utils;
    public SampleMecanumDrive drive;
    public ColorSensorClass colorSensorClassObj;
    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
        utils = new Utils(robot, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        new OpticalSensor(OpticalSensor.RobotType.COMPETITION, robot);
        colorSensorClassObj = new ColorSensorClass(robot.colorSensor);
        robot.opticalSensor.setPosition(new SparkFunOTOS.Pose2D(0,0, Math.toRadians(180)));
        robot.limelight.pipelineSwitch(0);
        robot.limelight.setPollRateHz(100);
        robot.limelight.start();
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
