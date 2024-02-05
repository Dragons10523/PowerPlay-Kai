package org.firstinspires.ftc.teamcode.Test;


import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;

@TeleOp(name="Robot Tele", group="Test")
public class RobotTelemetry extends OpMode {

    DcMotor FLM, FRM, BLM, BRM;
    DcMotorEx TEST;

//    VoltageSensor voltageSensor;

    IMU imu;


    @Override
    public void init() {
        FLM = hardwareMap.get(DcMotor.class, "FrontLeft");
        FRM = hardwareMap.get(DcMotor.class, "FrontRight");
        BLM = hardwareMap.get(DcMotor.class, "BackLeft");
        BRM = hardwareMap.get(DcMotor.class, "BackRight");
        imu = hardwareMap.get(IMU.class, "imu");

       // voltageSensor = hardwareMap.get(VoltageSensor.class, "voltageSensor");


        setDriveTrainPower(0,0);
        double xRotation = 0;  // enter the desired X rotation angle here.
        double yRotation = 0;  // enter the desired Y rotation angle here.
        double zRotation = 0;  // enter the desired Z rotation angle here.

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        // Now initialize the IMU with this mounting orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    @Override
    public void loop() {
//        double theta = gamepad1.left_stick_x * Math.PI * 2;
//        double angle = gamepad1.left_stick_y * Math.PI * 2;
//
//        double[] speeds = {
//                (drive + strafe),
//                (drive - strafe),
//                (drive - strafe),
//                (drive + strafe)};
        double rightPower = gamepad1.left_stick_y;
        double leftPower = gamepad1.right_stick_y;

        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);

        FLM.setPower(leftPower);
        FRM.setPower(rightPower);
        BLM.setPower(leftPower);
        BRM.setPower(rightPower);



        telemetry.addData("FrontLeftMotor", FLM.getPower());
        telemetry.addData("FrontRightMotor", FRM.getPower());
        telemetry.addData("BackLeftMotor", BLM.getPower());
        telemetry.addData("BackRightMotor", BRM.getPower());
        telemetry.addData("Angular Velocity", imu.getRobotAngularVelocity(AngleUnit.DEGREES));
        telemetry.addData("Theta", getHeading());
        telemetry.addData("Voltage", getBatteryVoltage());
       // telemetry.addData("BatteryLevel", voltageSensor.getVoltage());
        telemetry.update();
    }
    public void setDriveTrainPower(double leftPower, double rightPower){
        FLM.setPower(leftPower);
        FRM.setPower(rightPower);
        BLM.setPower(leftPower);
        BRM.setPower(rightPower);
    }
    public double getHeading(){
//        double strafe = Math.cos(theta) - Math.sin(theta);
//        double drive = Math.sin(angle) + Math.cos(angle);
//        strafe /= 2;
//        drive /= 2;
        Orientation Theta = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return Theta.secondAngle;
    }
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
