package org.firstinspires.ftc.teamcode.Susbsystem;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.OpModes.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class AutoUtils {
    RobotClass robot;
    SparkFunOTOS opticalSensor;
    public static final int WHEEL_RADIUS = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 / WHEEL_CIRCUMFERENCE_INCH;
    Telemetry telemetry;
    AprilTagPipeline aprilTagPipeline;

    public AutoUtils(RobotClass robot, Telemetry telemetry) {
        this.robot = robot;
        opticalSensor = robot.opticalSensor;
        this.telemetry = telemetry;
        aprilTagPipeline = new AprilTagPipeline(robot.webcamName, telemetry);
    }

    Map<RobotClass.MOTORS, Double> wheelSpeeds = new HashMap<>();

    public void moveToPosition(double x, double y) {

    }

    public void updateOpticalSensorToCameraDetections() {
        if (!aprilTagPipeline.getDetections().isEmpty()) {

            SparkFunOTOS.Pose2D updatedPosition = new SparkFunOTOS.Pose2D(closestAprilTagLocation[0], closestAprilTagLocation[1], opticalSensor.getPosition().h);
            opticalSensor.setPosition(updatedPosition);
        } else {
            telemetry.addLine("NO DETECTIONS");
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private final double kD = .01;

    @SuppressLint("DefaultLocale")
    public void AutoDrive(double targetDistance_INCH, double angle) {
        ElapsedTime timer = new ElapsedTime();

        angle = Math.toRadians(angle);
        double maxErrorAllowed = .1 * TICKS_PER_INCH;
        double kP = 0.01;
        double kI = 0.01;
        double kD = 0.01;
        double integral = 0;
        double maxIntegral = 2.0;

        double targetDistance = targetDistance_INCH * TICKS_PER_INCH;
        double previousTime = 0;
        double previousError = 0;
        while (!AutoControl.isStopRequested.getAsBoolean()) {
            double currentTime = timer.milliseconds();
            SparkFunOTOS.Pose2D Pose2D = robot.opticalSensor.getPosition();

            double error = (targetDistance - Math.sqrt(Math.pow((targetDistance * Math.cos(angle) - Pose2D.x), 2)
                    + (Math.pow((targetDistance * Math.sin(angle)) - Pose2D.y, 2))));

            double proportional = kP * error;
            //calculate power to the motor based on error
            integral += kI * (error * (currentTime - previousTime));
            integral = constrainDouble(0, maxIntegral, integral);
            // calculate power to the motor based on error over time and constrain
            // the value to maxIntegral
            double derivative = kD * (error - previousError) / (timer.milliseconds() - previousTime);
            // reduces power to the motor based on how quickly the robot approached the desired target
            double output = proportional + integral + derivative;

            double x = Math.cos(angle);
            double y = Math.sin(angle);
            //calculates the direction the robot must travel in a unit circle (x, y)
            double rotY = y * output;
            double rotX = x * output;
            //multiplies desired direction by corrected output value

            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY), 1.0);
            // calculates the absolute value of the maximum motor power

            wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, (rotY + rotX) / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, (rotY - rotX) / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, (rotY - rotX) / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, (rotY + rotX) / denominator);
            //sets motor power and normalizes ranges based on the maximum motor power
            telemetry.addData("proportional", proportional);
            telemetry.addData("integral", integral);
            telemetry.addData("derivative", derivative);
            telemetry.update();
            if (Math.abs(error) < maxErrorAllowed) {
                break;
            }
            previousTime = timer.milliseconds();
            previousError = error;
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
        while (!atTarget && !AutoControl.isStopRequested.getAsBoolean());
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
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        setCoastWheelBehavior();
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private double getDerivative(double error, double targetDistance, double angle) {
        double previousError = error;
        SparkFunOTOS.Pose2D Pose2D = robot.opticalSensor.getPosition();
        error = (targetDistance - Math.sqrt(Math.pow((targetDistance * Math.cos(angle) - Pose2D.x), 2)
                + (Math.pow((targetDistance * Math.sin(angle)) - Pose2D.y, 2))));
        return (error - previousError) / (previousError + 0.01);
    }

    public void UpdateWheelPowers() {
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT));
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT));
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT));
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT));
    }

    public void setStopWheelBehavior() {
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setCoastWheelBehavior() {
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double constrainDouble(double lowerBound, double upperBound, double val) {
        val = Math.max(val, lowerBound);
        val = Math.min(val, upperBound);
        return val;
    }

    public AprilTagPoseFtc getClosestAprilTagLocation() {
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getDetections();
        if (detections.isEmpty()) {
            return null;
        }
        double closestTagRange = 100;
        int closestTagID = -1;
        for (AprilTagDetection detection : detections) {
            double tagRange = detection.ftcPose.range;
            if (tagRange < closestTagRange) {
                closestTagRange = tagRange;
                closestTagID = detection.id;
            }
        }
        if(closestTagID == -1){
            return null;
        }
        else{
            return detections.get(closestTagID).ftcPose;
        }
    }

}
