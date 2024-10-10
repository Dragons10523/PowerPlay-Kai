package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.AutoUtils;

import java.util.function.BooleanSupplier;

@Autonomous(name = "Auto_Blue_Left")
public class AutoControlBlueLeft extends AutoControl {

    private final Control.FieldSide fieldSide = Control.FieldSide.BLUE_LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initialize();

        waitForStart();

        autoUtils.AutoDrive(10,10);

        while(!isStopRequested()){
            //aprilTagPipeline.updateAprilTagPipeline();
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %6.2f %6.2f %6.2f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0.0);
    }
}