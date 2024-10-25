package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Auto.AutoUtils;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;

public class AutoControl extends LinearOpMode {
    public RobotClass robot;
    public AutoUtils autoUtils;
    public SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public boolean getStopRequested(){
        return isStopRequested();
    }

    public void initialize(){
        OpticalSensor opticalSensorObj = new OpticalSensor();
        robot = new RobotClass(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        autoUtils = new AutoUtils(robot, telemetry);
        opticalSensorObj.configureOtos(robot);
        //^^^^ fixes inertial odometry  drift
    }
    public Pose2d getCurrentAdjustedPosition(){
        SparkFunOTOS.Pose2D pos = robot.opticalSensor.getPosition();
        double adjustedX = -pos.y;
        double adjustedY = pos.x;

        return new Pose2d(adjustedX, adjustedY, pos.h);
    }
}
