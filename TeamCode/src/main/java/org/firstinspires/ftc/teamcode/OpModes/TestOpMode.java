package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.Arrays;
import java.util.List;


@TeleOp
public class TestOpMode extends AutoControl {
    private final Utils.FieldSide fieldSide = Utils.FieldSide.BLUE_LEFT;

    @SuppressLint("DefaultLocale")
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        super.initialize();
//        Thread t1 = new Thread() {
//            public void run() {
//                while (!isStopRequested()) {
//
//
//                    telemetry.update();
//                }
//            }
//        };
//        t1.start();

        waitForStart();
        robot.opticalSensor.resetTracking();
        robot.opticalSensor.setPosition(new SparkFunOTOS.Pose2D(0,0, Math.toRadians(0)));
        robot.opticalSensor.calibrateImu();
//        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
//                .strafeRight(10)
//                .build();
//
//        drive.followTrajectorySequence(traj1);

        while (!isStopRequested()) {
            LLResult result = robot.limelight.getLatestResult();
            robot.limelight.updateRobotOrientation(Math.toDegrees(robot.getHeading()));
            if (result != null) {
                if (result.isValid()) {
                    Pose3D pos_MT2 = result.getBotpose_MT2();
                    Position pos = pos_MT2.getPosition().toUnit(DistanceUnit.INCH);
                    telemetry.addData("pos_MT2", "XY: %.2f %.2f", pos.x, pos.y);;
                } else {
                    telemetry.addLine("result is not valid");
                }
            } else {
                telemetry.addLine("No data available");
            }
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
    }


}
