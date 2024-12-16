package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Auto.AutoUtils;
import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;

public class AutoControl extends LinearOpMode {
    public RobotClass robot;
    public AutoUtils autoUtils;
    public SampleMecanumDrive drive;
    public Limelight limelightObj;
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
        new OpticalSensor(OpticalSensor.RobotType.COMPETITION, robot);
        limelightObj = new Limelight(robot);
    }
    public void simpleInit(){
        robot = new RobotClass(hardwareMap);
    }
    public void cameraLocalization(){

        robot.opticalSensor.setPosition(new SparkFunOTOS.Pose2D(0, 0, autoUtils.getCameraHeading()));

        Thread thread1 = new Thread() {
            public void run() {
                while (!isStopRequested()) {
                    SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
                    LLResult result = robot.limelight.getLatestResult();
                    telemetry.addData("XYH: ", "%.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h);
                    telemetry.addData("successfulLocalizations",  autoUtils.getSuccessfulLocalizationCount());
                    telemetry.addData("validResult?", result.isValid());
                    telemetry.update();
                }
            }
        };
        thread1.start();
        Thread thread2 = new Thread() {
            public void run() {
                while(!isStopRequested()) {
                    autoUtils.updateOpticalSensorToPoseEstimateCamera(5);
                }
            }
        };
        thread2.start();

        telemetry.addLine("sleeping 1 second");
        telemetry.update();
        sleep(1000);

    }
}
