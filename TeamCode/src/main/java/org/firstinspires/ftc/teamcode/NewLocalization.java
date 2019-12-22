package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;


/**
 * All distance sensors used for localization are in an array with these values
 * 0=left
 * 1=right
 * 2=rear
 * 3=left
 */

public class NewLocalization extends LinearOpMode {

    DistanceSensor[] sensors;

    double X = 0, Y = 0;


    @Override
    public void runOpMode() throws InterruptedException {

    }

    void updatePosition(){
        double[] rawDistance = {0, 0, 0, 0};
        for(int i =0; i < sensors.length; i++){
            rawDistance[i] = sensors[i].getDistance(DistanceUnit.INCH);
        }

        final double lastX;
        final double lastY;



    }

    void init(HardwareMap hwmap, boolean limitS){
        HardwareConfig robot = new HardwareConfig(hwmap);
        robot.initializeDriveTrain();
        robot.initializeDistanceSensors();
        robot.initializeIMU();
        if(limitS) {
            robot.initializeLimitSwitches();
            DigitalChannel limitSwitchh = robot.limitLeft;
        }

        DriveTrain dt = new DriveTrain(robot.frontLeft, robot.frontRight, robot.rearRight, robot.rearLeft);
        MecanumDrive mecanums = new MecanumDrive(dt);
        Encoders encoders = new Encoders(dt);

        encoders.resetEncoders();

        System.arraycopy(robot.distanceSensors, 0, sensors, 0, robot.distanceSensors.length);

        Servo leftClaw = robot.leftClaw;
        Servo rightClaw = robot.rightClaw;

        robot.initializeIMU();

        BNO055IMU imu = robot.imu;

        double angleError = -imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES).firstAngle;
    }


}
