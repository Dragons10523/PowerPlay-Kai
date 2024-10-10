package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;

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
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0.0);
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0.0);
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0.0);
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0.0);
    }
}