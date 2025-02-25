package org.firstinspires.ftc.teamcode.Auto;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class AutoUtils {
    ElapsedTime time = new ElapsedTime();
    RobotClass robot;
    public static final int WHEEL_RADIUS = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS;
    public static final double INCHES_PER_METER = 39.37;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 / WHEEL_CIRCUMFERENCE_INCH;

    BooleanSupplier isStopRequested;

    final int highBasketHeightEncoder = 1750;
    Telemetry telemetry;
    AutoControl autoControl;

    public AutoUtils(RobotClass robot, Telemetry telemetry, BooleanSupplier isStopRequested) {
        this.robot = robot;
        this.telemetry = telemetry;
        autoControl = new AutoControl();
        this.isStopRequested = isStopRequested;
    }

    Map<RobotClass.MOTORS, Double> wheelSpeeds = new HashMap<>();

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
            // the value to maxIntegrala
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

    public void sampleWiggle() {
        double wiggleStartTime = time.seconds();
        while (wiggleStartTime + 0.45 > time.seconds()) {
            robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-0.75);
            robot.drivetrain.simpleDrive(-1);
        }
        wiggleStartTime = time.seconds();
        while (wiggleStartTime + 0.2 > time.seconds()) {
            robot.drivetrain.simpleDrive(1);
        }
        robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0);
        robot.drivetrain.simpleDrive(0);
    }

    //Performs transition from intake to bucket
    public void intakeTransition() {

        //set arm position in
        armExtension(Utils.ArmState.IN);
        //set gate closed
        robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(0.4);
        //bucket pos
        robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0.37);
        //retract vertical slide
        Thread t2 = new Thread() {
            @Override
            public void run() {
                double startTime = time.seconds();
                while (startTime + 0.5 > time.seconds()) {
                    robot.Motors.get(RobotClass.MOTORS.LIFT).setPower(-.3);
                }
                robot.Motors.get(RobotClass.MOTORS.LIFT).setPower(0);
            }
        };
        t2.start();
        //retract arm
        Thread t3 = new Thread() {
            @Override
            public void run() {
                armFlip(Utils.ArmFlipState.UP, 0.6);
            }
        };
        t3.start();
        double startTime = time.seconds();
        while (startTime + 0.5 > time.seconds()) {
            boolean isWaiting = true;
        }
        robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(0.8);
        robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);

        //ENDS WITH INTAKE SPINNING
        sampleWiggle();
    }

    //Moves jointed arm to target ArmFlipState
    public void armFlip(Utils.ArmFlipState state, double power) {
        double startTime;
        int targetPos;
        int currentPos;
        int distanceToTarget;
        switch (state) {
            case GROUND:
                robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPos = 3900;
                currentPos = robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition();
                distanceToTarget = targetPos - currentPos;
                startTime = time.seconds();
                while (Math.abs(distanceToTarget) > 20) {
                    currentPos = robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition();
                    distanceToTarget = targetPos - currentPos;
                    double powerOut = (double) distanceToTarget / 1500;
                    telemetry.addData("powerOut", powerOut);
                    powerOut /= power;
                    powerOut = powerOut > 0 ? Math.max(0.2, powerOut) : Math.min(-0.2, powerOut);
                    robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(powerOut);
                    telemetry.addData("adjustedPowerOut", powerOut);
                    telemetry.addData("targetPos", targetPos);
                    telemetry.addData("distanceToTarget", distanceToTarget);
                    telemetry.addData("currentPos", currentPos);
                    telemetry.update();
                    if (startTime + 1 < time.seconds()) {
                        break;
                    }
                }
                break;
            case UP:
                robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPos = 150;
                currentPos = robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition();
                distanceToTarget = targetPos - currentPos;
                startTime = time.seconds();
                while (Math.abs(distanceToTarget) > 200) {
                    currentPos = robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition();
                    distanceToTarget = targetPos - currentPos;
                    double powerOut = (double) distanceToTarget / 1500;
                    telemetry.addData("powerOut", powerOut);
                    powerOut /= power;
                    powerOut = powerOut > 0 ? Math.max(0.2, powerOut) : Math.min(-0.2, powerOut);
                    robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(powerOut);
                    telemetry.addData("adjustedPowerOut", powerOut);
                    telemetry.addData("targetPos", targetPos);
                    telemetry.addData("distanceToTarget", distanceToTarget);
                    telemetry.addData("currentPos", currentPos);
                    telemetry.update();
                    if (startTime + 1 < time.seconds()) {
                        break;
                    }
                }
                startTime = time.seconds();
                while (startTime + 0.2 > time.seconds()) {
                    robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(-.4);
                }
                robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(0);
                robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case MIDDLE:
                robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPos = 1000;
                currentPos = robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition();
                distanceToTarget = targetPos - currentPos;
                startTime = time.seconds();
                while (Math.abs(distanceToTarget) > 20) {
                    currentPos = robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).getCurrentPosition();
                    distanceToTarget = targetPos - currentPos;
                    double powerOut = (double) distanceToTarget / 1000;
                    powerOut = powerOut > 0 ? Math.max(0.2, Math.max(power, powerOut)) : Math.min(-0.2, Math.min(-power, powerOut));
                    robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(powerOut);
                    if (startTime + 2 < time.seconds()) {
                        break;
                    }
                }
                break;
        }
        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motors.get(RobotClass.MOTORS.ARM_FLIP).setPower(0);
    }

    //Moves to liftState
    //GROUND state powers in down direction for 2 seconds
    public void verticalSlide(Utils.LiftState liftState) {
        double startTime = time.seconds();
        int currentPos = robot.Motors.get(RobotClass.MOTORS.LIFT).getCurrentPosition();
        int previousPos = 0;
        setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        switch (liftState) {
            case HIGH:
                int targetPos = highBasketHeightEncoder;
                setLiftTargetPos(targetPos);
                while (currentPos > targetPos + 10 || currentPos < targetPos - 10 && !autoControl.isStopRequested()) {
                    currentPos = -robot.Motors.get(RobotClass.MOTORS.LIFT).getCurrentPosition();
                    //calculates powerOut based on difference between goal ticks / 300
                    double powerOut = (double) (targetPos - currentPos) / 200.0;
                    telemetry.addData("powerOut", powerOut);
                    telemetry.addData("targetPos", targetPos);
                    telemetry.addData("currentPos", currentPos);
                    telemetry.addData("mode", robot.Motors.get(RobotClass.MOTORS.LIFT).getMode());
                    //Limits power out between -0.4 > x > 0.6
                    if (powerOut > 0) {
                        powerOut = Math.max(powerOut, 0.6);
                    } else {
                        powerOut = Math.min(powerOut, -0.4);
                    }
                    telemetry.addData("actualPower", powerOut);
                    telemetry.update();
                    setLiftPower(powerOut);
                    //Time out 3 seconds
                    if (startTime + 2 < time.seconds() || autoControl.isStopRequested()) {
                        break;
                    }
                    if (previousPos == currentPos && startTime + 0.3 < time.seconds()) {
                        break;
                    }
                    previousPos = currentPos;
                }
                setLiftPower(0.2);
                break;
            case GROUND:
                while (startTime + 2 > time.seconds()) {
                    setLiftPower(-1);
                    if (autoControl.isStopRequested()) {
                        break;
                    }
                }
                setLiftPower(0);
                robot.Motors.get(RobotClass.MOTORS.LIFT).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
    }

    public void scorePiece(double time) {
        ElapsedTime elapsedTime = new ElapsedTime();
        if (robot.Motors.get(RobotClass.MOTORS.LIFT).getCurrentPosition() < highBasketHeightEncoder - 100) {
            verticalSlide(Utils.LiftState.HIGH);
        }
        double startTime = elapsedTime.seconds();
        while (startTime + time > elapsedTime.seconds() && !autoControl.isStopRequested()) {
            robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0.85);
        }
        robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0.39);
    }

    Thread inThread = new Thread() {
        public void run() {
            double startTime = time.seconds();
            while (startTime + 2 > time.seconds() && !isInterrupted()) {
                robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(Utils.arm_leftPos[0]);
                robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(Utils.arm_rightPos[0]);
            }
        }
    };
    Thread outThread = new Thread() {
        public void run() {
            double startTime = time.seconds();
            while (startTime + 2 > time.seconds() && !isInterrupted()) {
                robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(Utils.arm_leftPos[Utils.arm_leftPos.length - 1]);
                robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(Utils.arm_rightPos[Utils.arm_rightPos.length - 1]);
            }
        }
    };

    public void armExtension(Utils.ArmState armState) {
        switch (armState) {
            case IN:
//                if (outThread.isAlive()) {
//                    outThread.interrupt();
//                }
//                if (!inThread.isAlive()) {
//                    inThread.start();
//                }
                robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(Utils.arm_leftPos[0]);
                robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(Utils.arm_rightPos[0]);
                break;
            case EXTENDED:
//                if (inThread.isAlive()) {
//                    inThread.interrupt();
//                }
//                if (!outThread.isAlive()) {
//                    outThread.start();
//                }
                robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(Utils.arm_leftPos[Utils.arm_leftPos.length - 1]);
                robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(Utils.arm_rightPos[Utils.arm_rightPos.length - 1]);
                break;
        }
    }

    int successfulLocalizationCount = 0;

    public int getSuccessfulLocalizationCount() {
        return successfulLocalizationCount;
    }

    double startTime = 0;
    double heading_RADIANS = 0;
    int pullCount = 0;

    public double getCameraHeading() {
        startTime = time.seconds();

        while (!autoControl.isStopRequested()) {
            LLResult result = robot.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                pullCount++;
                heading_RADIANS += result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS);
                if (pullCount >= 10) {
                    return heading_RADIANS / pullCount;
                }

            }
            if (startTime + 5 < time.seconds()) {
                break;
            }
        }
        return 0;
    }

    public boolean updateOpticalSensorToPoseEstimateCamera() {
        LLResult result = robot.limelight.getLatestResult();
        SparkFunOTOS.Pose2D pos_Sensor = robot.opticalSensor.getPosition();
        robot.limelight.updateRobotOrientation(Math.toDegrees(robot.getHeading()));

        if (result != null && result.isValid()) {
            Position pose2D = result.getBotpose_MT2().getPosition().toUnit(DistanceUnit.INCH);
            double[] stdDevMt2 = result.getStddevMt2();
            if (stdDevMt2[0] + stdDevMt2[1] < 1) { //xy
                robot.opticalSensor.setPosition(new SparkFunOTOS.Pose2D(pose2D.toUnit(DistanceUnit.INCH).x, pose2D.toUnit(DistanceUnit.INCH).y, robot.getHeading()));
                successfulLocalizationCount++;
                return true;
            } else {
                telemetry.addLine("stdDevMt2 > 1 in");
                telemetry.addData("x", stdDevMt2[0]);
                telemetry.addData("y", stdDevMt2[1]);
                telemetry.addData("total", stdDevMt2[0] + stdDevMt2[1]);
            }
        } else {
            telemetry.addLine("No Data Available");
            return false;
        }
        return false;
    }

    public boolean inRange(double v1, double v2, double range) {
        double lowerRange = v1 - range;
        double upperRange = v1 + range;

        return lowerRange <= v2 && upperRange >= v2;
    }

    public void setLiftMode(DcMotor.RunMode runMode) {
        robot.Motors.get(RobotClass.MOTORS.LIFT).setMode(runMode);
    }

    public void setLiftTargetPos(int targetPos) {
        robot.Motors.get(RobotClass.MOTORS.LIFT).setTargetPosition(targetPos);
    }

    public void setLiftPower(double power) {
        robot.Motors.get(RobotClass.MOTORS.LIFT).setPower(power);
    }

    public double constrainDouble(double lowerBound, double upperBound, double val) {
        val = Math.max(val, lowerBound);
        val = Math.min(val, upperBound);
        return val;
    }
}
