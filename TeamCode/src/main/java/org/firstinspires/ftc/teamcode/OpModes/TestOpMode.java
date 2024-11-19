package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;


@TeleOp
public class TestOpMode extends AutoControl {
    private final Utils.FieldSide fieldSide = Utils.FieldSide.BLUE_LEFT;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        super.initialize();

        while(!isStarted()){
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
        waitForStart();
        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(90));
        robot.opticalSensor.setPosition(pos);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .addTemporalMarker(0,()->{
                    Thread t1 = new Thread() {
                        public void run() {
                            while(!isStopRequested()){
                                SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
                                telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
                                telemetry.update();
                            }
                        }
                    };
                    t1.start();
                })
                .lineToConstantHeading(new Vector2d(20,0))
                .waitSeconds(5)
                .lineToConstantHeading(new Vector2d(0,0))
                .build();

        drive.followTrajectorySequence(traj);

        while(!isStopRequested()){
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
        //drive.followTrajectorySequence(trajSeq);
//        while(opModeIsActive()){
//            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
//            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
//            telemetry.update();
//        }
    }


}
