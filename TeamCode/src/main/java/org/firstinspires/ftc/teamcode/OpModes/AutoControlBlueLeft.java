package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.AutoUtils;

import java.util.function.BooleanSupplier;

@Autonomous(name = "Auto_Blue_Left")
public class AutoControlBlueLeft extends OpMode {
    public RobotClass robot;
    private final Control.FieldSide fieldSide = Control.FieldSide.BLUE_LEFT;
    private static boolean stop = false;
    public static BooleanSupplier isStopRequested = () -> stop;
    AutoUtils autoUtils;

    @Override
    public void init() {
        stop = false;
        robot = new RobotClass(hardwareMap);
        autoUtils = new AutoUtils(robot, telemetry);
        new OpticalSensor(robot, fieldSide);
        robot.setDirection();
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    public void start() {
//        autoUtils.AutoDrive(10,10);
//        autoUtils.AutoTurn(0);

    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        //aprilTagPipeline.updateAprilTagPipeline();
        SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
        telemetry.addLine(String.format("XYH %6.2f %6.2f %6.2f", pose2D.x, pose2D.y, pose2D.h));
        telemetry.addData("isStopRequested", isStopRequested.getAsBoolean());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot == null) return; // ensures that stop() is not called before initialization
        stop = true;
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0.0);
        requestOpModeStop();
    }


}