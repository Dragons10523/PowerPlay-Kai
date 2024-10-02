package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.AutoUtils;

import java.util.function.BooleanSupplier;

@Autonomous(name = "Auto_Red_Right")
public class AutoControlRedRight extends OpMode {
    public RobotClass robot;
    private final Control.FieldSide fieldSide = Control.FieldSide.RED_RIGHT;
    private static boolean stop = false;
    public static BooleanSupplier isStopRequested = () -> stop;
    AutoUtils autoUtils;

    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
        autoUtils = new AutoUtils(robot, telemetry);
        robot.setDirection();
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    public void start() {
        autoUtils.moveToPosition(10,10,90);
    }

    @Override
    public void loop() {
        //aprilTagPipeline.updateAprilTagPipeline();
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