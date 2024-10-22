package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Susbsystem.AutoUtils;
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
        robot = new RobotClass(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        autoUtils = new AutoUtils(robot, telemetry);
        OpticalSensor.configureOtos(robot);
        robot.opticalSensor.calibrateImu();
        //^^^^ fixes inertial odometry  drift
        robot.initMotorsProto();
    }

}
