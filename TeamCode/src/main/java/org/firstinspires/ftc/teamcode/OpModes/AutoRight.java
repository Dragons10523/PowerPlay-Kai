package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Auto.AutoUtils;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(name = "Auto_Right_park")
public class AutoRight extends AutoControl {
    private final Utils.FieldSide fieldSide = Utils.FieldSide.BLUE_RIGHT;
    AutoUtils autoUtils;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        super.initialize();

        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(-14, 62, Math.toRadians(270));

        robot.opticalSensor.setPosition(pos);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-14, 62, Math.toRadians(270)))
                .strafeRight(20)
                .build();

        waitForStart();

        drive.followTrajectorySequence(traj1);

        while(!isStopRequested()){
            //aprilTagPipeline.updateAprilTagPipeline();
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %6.2f %6.2f %6.2f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
    }


}