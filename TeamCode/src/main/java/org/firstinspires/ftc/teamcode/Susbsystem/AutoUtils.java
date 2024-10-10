package org.firstinspires.ftc.teamcode.Susbsystem;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Camera.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.HashMap;
import java.util.Map;

public class AutoUtils {
    RobotClass robot;
    SparkFunOTOS opticalSensor;
    public static final int WHEEL_RADIUS = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 / WHEEL_CIRCUMFERENCE_INCH;
    public static final int CAMERA_OFFSET_X = 0;
    public static final int CAMERA_OFFSET_Y = 0;
    Telemetry telemetry;
    AprilTagPipeline aprilTagPipeline;
    AutoControl autoControl;

    public AutoUtils(RobotClass robot, Telemetry telemetry) {
        this.robot = robot;
        opticalSensor = robot.opticalSensor;
        this.telemetry = telemetry;
        aprilTagPipeline = new AprilTagPipeline(robot.webcamName, telemetry);
        autoControl = new AutoControl();
    }

    Map<RobotClass.MOTORS, Double> wheelSpeeds = new HashMap<>();


    public void updateOpticalSensorToCameraDetection(int goalTagID) {
        if (!aprilTagPipeline.getDetections().isEmpty()) {
            int closestID = aprilTagPipeline.getClosestAprilTagID();
            if (closestID == goalTagID) {
                //TODO: set origin based on aprilTag, ensure that aprilTag is placed accurately
                AprilTagPoseFtc poseFtc = aprilTagPipeline.getClosestAprilTagLocation();
                SparkFunOTOS.Pose2D updatedPosition = new SparkFunOTOS.Pose2D(poseFtc.x + CAMERA_OFFSET_X, poseFtc.y + CAMERA_OFFSET_Y, opticalSensor.getPosition().h);
                opticalSensor.setPosition(updatedPosition);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

        } else {
            telemetry.addLine("NO DETECTIONS");
            telemetry.update();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    @SuppressLint("DefaultLocale")
    public void AutoDrive(double targetDistance_INCH_X, double targetDistance_INCH_Y) {
        ElapsedTime timer = new ElapsedTime();


        double maxErrorAllowed = .2;
        double kP = 0.05;
        double kI = 0.15;
        double kD = 0.02;
        double integralX = 0;
        double integralY = 0;
        double maxIntegral = .75;
        double previousTime = 0;
        double previousErrorX = 0;
        double previousErrorY = 0;
        while (!autoControl.getStopRequested()) {
            double currentTime = timer.seconds();
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();

            double errorX = targetDistance_INCH_X - (-pose2D.x);
            double errorY = targetDistance_INCH_Y - (-pose2D.y);

            double proportionalX = kP * errorX;
            double proportionalY = kP * errorY;

            //calculate power to the motor based on error
            integralX += kI * (errorX * (currentTime - previousTime));
            integralY += kI * (errorY * (currentTime - previousTime));

            integralX = constrainDouble(-maxIntegral, maxIntegral, integralX);
            integralY = constrainDouble(-maxIntegral, maxIntegral, integralY);
            // calculate power to the motor based on error over time and constrain
            // the value to maxIntegral
            double derivativeX = kD * (errorX - previousErrorX) / (timer.seconds() - previousTime);
            double derivativeY = kD * (errorY - previousErrorY) / (timer.seconds() - previousTime);
            // reduces power to the motor based on how quickly the robot approached the desired target
            double outputX = proportionalX + integralX + derivativeX;
            double outputY = proportionalY + integralY + derivativeY;

            //multiplies desired direction by corrected output value

            double denominator = Math.max(Math.abs(outputX) + Math.abs(outputY), 1.0);
            // calculates the absolute value of the maximum motor power

            wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, (outputY + outputX) / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, (outputY - outputX) / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, (outputY - outputX) / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, (outputY + outputX) / denominator);
            //sets motor power and normalizes ranges based on the maximum motor power
            telemetry.addData("errorX", errorX);
            telemetry.addData("errorY", errorY);
            telemetry.addData("outputX", outputX);
            telemetry.addData("outputY", outputY);
            telemetry.addData("currentTime", currentTime);
            telemetry.addData("previousTime", previousTime);
            telemetry.addData("proportionalX", proportionalX);
            telemetry.addData("proportionalY", proportionalY);
            telemetry.addData("integralX", integralX);
            telemetry.addData("integralY", integralY);
            telemetry.addData("derivativeX", derivativeX);
            telemetry.addData("derivativeY", derivativeY);
            telemetry.update();
            if (Math.abs(errorX) < maxErrorAllowed && Math.abs(errorY) < maxErrorAllowed) {
                break;
            }
            previousTime = timer.seconds();
            previousErrorX = errorX;
            previousErrorY = errorY;

            UpdateWheelPowers();
        }
        stopMotors();
    }

    public void AutoTurn(double targetAngle) {

        boolean atTarget = false;
        double angularDistance = 0;
        int turnVal = 0;
        do {
            double currentAngle = Math.toDegrees(robot.getHeading());

            if (currentAngle < 180) {
                currentAngle += 360;
            }
            //adjusts ranges to be from 0 - 360 instead of 180 - -180
            if (currentAngle <= targetAngle) {
                if (targetAngle - currentAngle <= 180) {
                    //Rotate Clockwise
                    angularDistance = targetAngle - currentAngle;
                    turnVal = 1;
                } else {
                    //Rotate Counter-Clockwise
                    angularDistance = 360 - targetAngle + currentAngle;
                    turnVal = -1;
                }
            } else {//If currentAngle > targetAngle so the opposite of the other if statement
                if (currentAngle - targetAngle < 180) {
                    //Rotate Counter-Clockwise
                    angularDistance = currentAngle - targetAngle;
                    turnVal = -1;
                } else {
                    //Rotate Clockwise
                    angularDistance = 360 - currentAngle + targetAngle;
                    turnVal = 1;
                }
            }
            double powerReduce = angularDistance / 90;

            powerReduce = Math.max(powerReduce, 0.2);
            powerReduce = Math.min(powerReduce, 0.9);

            wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, -turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, -turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, turnVal * powerReduce);

            UpdateWheelPowers();

            if (angularDistance < 0.5) atTarget = true;
        }
        while (!atTarget);
        stopMotors();
    }

    public void simplePower(double power) {
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, power);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, power);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, power);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, power);
        UpdateWheelPowers();
    }

    public void stopMotors() {
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, 0.0);
        UpdateWheelPowers();

        setStopWheelBehavior();
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {

        }
        setCoastWheelBehavior();
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {

        }
    }

    public void UpdateWheelPowers() {
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT));
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT));
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT));
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT));
    }

    public void setStopWheelBehavior() {
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setCoastWheelBehavior() {
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double constrainDouble(double lowerBound, double upperBound, double val) {
        val = Math.max(val, lowerBound);
        val = Math.min(val, upperBound);
        return val;
    }
}
