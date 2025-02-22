package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;

public class OpticalSensor {
    RobotClass robot;
    public static enum RobotType{
        COMPETITION,
        PROTO
    }
    public OpticalSensor(RobotType robotType, RobotClass robot){
        if(robotType == RobotType.COMPETITION){
            configureOtosComp(robot);
        }
        else{
            configureOtosProto(robot);
        }
        this.robot = robot;
    }

    public void configureOtosProto(RobotClass robot){
        robot.opticalSensor.setLinearUnit(DistanceUnit.INCH);
        robot.opticalSensor.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0,0, Math.toRadians(-2.50 + 180));
        robot.opticalSensor.setOffset(offset);

        robot.opticalSensor.setLinearScalar(1.0);
        robot.opticalSensor.setAngularScalar(1.0);

        robot.opticalSensor.calibrateImu();


        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        robot.opticalSensor.getVersionInfo(hwVersion, fwVersion);
    }
    public void configureOtosComp(RobotClass robot){
        robot.opticalSensor.setLinearUnit(DistanceUnit.INCH);
        robot.opticalSensor.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0,0, Math.toRadians(-2.50 + 180));
        robot.opticalSensor.setOffset(offset);

        robot.opticalSensor.setLinearScalar(1.0);
        robot.opticalSensor.setAngularScalar(1.0);

        robot.opticalSensor.calibrateImu();

        robot.opticalSensor.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0,0,0);
        robot.opticalSensor.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        robot.opticalSensor.getVersionInfo(hwVersion, fwVersion);
    }
    public double getDisplacementFromTarget(double targetX, double targetY){
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        double displacementX = targetX - currentPos.x;
        double displacementY = targetY - currentPos.y;
        return Math.sqrt((Math.pow(displacementX, 2) + Math.pow(displacementY, 2)));
    }
}
