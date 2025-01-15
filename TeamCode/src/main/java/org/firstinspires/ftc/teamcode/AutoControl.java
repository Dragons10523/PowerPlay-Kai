package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Auto.AutoUtils;
import org.firstinspires.ftc.teamcode.Auto.TrajectoryHandler;
import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;

public class AutoControl extends LinearOpMode {
    public RobotClass robot;
    public AutoUtils autoUtils;
    public SampleMecanumDrive drive;
    public OpticalSensor opticalSensorClass;
    public Limelight limelightObj;
    public TrajectoryHandler trajectoryHandler;
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
        trajectoryHandler = new TrajectoryHandler(robot, drive, autoUtils);
        opticalSensorClass = new OpticalSensor(OpticalSensor.RobotType.COMPETITION, robot);
        limelightObj = new Limelight(robot);
        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void simpleInit(){
        robot = new RobotClass(hardwareMap);
        autoUtils = new AutoUtils(robot, telemetry);
    }

    public void initialHeading(double heading_RADIANS, boolean doCheckHeading){
        if(doCheckHeading){
            double testHeading = autoUtils.getCameraHeading();
            if(autoUtils.inRange(testHeading, heading_RADIANS, Math.PI/2)){
                robot.opticalSensor.setPosition(new SparkFunOTOS.Pose2D(0,0, testHeading));
            }
        }
        else{
            robot.opticalSensor.setPosition(new SparkFunOTOS.Pose2D(0,0, heading_RADIANS));
        }

    }
    public void end(){

    }
}
