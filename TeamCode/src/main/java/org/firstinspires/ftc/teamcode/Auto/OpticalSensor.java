package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.OpModes.AutoControlBlueLeft;
import org.firstinspires.ftc.teamcode.RobotClass;

public class OpticalSensor {

    public static void configureOtos(RobotClass robot){
        robot.opticalSensor.setLinearUnit(DistanceUnit.INCH);
        robot.opticalSensor.setAngularUnit(AngleUnit.DEGREES);

        //set offset from center of robot IF sensor is not centered
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0,0,-2.50);
        robot.opticalSensor.setOffset(offset);

        SparkFunOTOS.Pose2D startingPos = new SparkFunOTOS.Pose2D(0,0,0);
        robot.opticalSensor.setPosition(startingPos);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        //actual value 3600 reported value 3598

        robot.opticalSensor.setAngularScalar(1.0);
        robot.opticalSensor.setLinearScalar(1.0);
        //33 1/8
        //0.62
        //actual value 331.25 read value 331.87
        //sensor reports error 0.001%
        robot.opticalSensor.getVersionInfo(hwVersion, fwVersion);

    }
    private double[] fieldSidePosition(){
        return null;
    }
}
