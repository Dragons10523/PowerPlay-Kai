package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.AutoUtils;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.function.BooleanSupplier;

@Autonomous(name = "Auto_Red_Right")
public class AutoControlRedRight extends AutoControl {
    public RobotClass robot;
    private final Utils.FieldSide fieldSide = Utils.FieldSide.RED_RIGHT;
    AutoUtils autoUtils;

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
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0.0);
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0.0);
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0.0);
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0.0);
    }

}